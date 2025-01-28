using UnityEngine;
using System.Collections.Generic;
using UnityEngine.UIElements;

#if UNITY_EDITOR
using UnityEditor;
#endif

//Attach to the parent

[ExecuteAlways]
public class OBBGenerator : MonoBehaviour
{
    [Header("Settings")]
    public bool RuntimeUpdates = true;
    public float UpdateInterval = 0.25f;

    [SerializeField] private GameObject obb;
    [SerializeField] private BoxCollider boxCollider;
    private Vector3[] prevLocalPositions;
    private Quaternion[] prevLocalRotations;
    private float updateTimer;


    private List<Vector3> vertices = new List<Vector3>();

    void Start() => InitializeOBB();

    void OnEnable()
    {
#if UNITY_EDITOR
        EditorApplication.update += EditorUpdate;
#endif
        EnsureOBBReference();
    }

    void OnDisable()
    {
#if UNITY_EDITOR
        EditorApplication.update -= EditorUpdate;
#endif
    }

    void Update()
    {
        if (Application.isPlaying && RuntimeUpdates)
        {
            updateTimer += Time.deltaTime;
            if (updateTimer >= UpdateInterval)
            {
                CheckForChanges();
                updateTimer = 0;
            }
        }
    }

#if UNITY_EDITOR
    void EditorUpdate()
    {
        if (!Application.isPlaying)
        {
            CheckForChanges();
        }
    }
#endif

    void EnsureOBBReference()
    {
        // Try to find existing OBB if reference was lost
        if (obb == null && transform.childCount > 0)
        {
            obb = transform.Find("Dynamic OBB")?.gameObject;
            if (obb != null) boxCollider = obb.GetComponent<BoxCollider>();
        }

        // Create new OBB if still missing
        if (obb == null) InitializeOBB();
    }

    void InitializeOBB()
    {
        // Check for existing OBB first
        Transform existingOBB = transform.Find("Dynamic OBB");
        if (existingOBB != null)
        {
            obb = existingOBB.gameObject;
            boxCollider = obb.GetComponent<BoxCollider>();
            return;
        }

        // Create new OBB
        obb = new GameObject("Dynamic OBB");
        obb.transform.SetParent(transform);
        obb.transform.localPosition = Vector3.zero;
        boxCollider = obb.AddComponent<BoxCollider>();

        // Serialize reference for persistence
#if UNITY_EDITOR
        if (!Application.isPlaying)
            EditorUtility.SetDirty(this);
#endif
    }

    void CheckForChanges()
    {
        if (LocalTransformsChanged())
        {
            GenerateOBB();
            StoreCurrentTransforms();
        }
    }

    void GenerateOBB()
    {
        EnsureOBBReference();

        MeshFilter[] meshFilters = GetComponentsInChildren<MeshFilter>();
        vertices = CollectLocalVertices(meshFilters);

        if (vertices.Count == 0) return;

        Vector3 centroid = ComputeCentroid(vertices);
        Matrix3x3 covariance = ComputeCovarianceMatrix(vertices, centroid);

        Matrix3x3 eigenvectors = Matrix3x3.identity;
        Vector3 eigenvalues = Vector3.zero;
        Jacobi(covariance, out eigenvectors, out eigenvalues);

        SortEigenvectors(ref eigenvalues, ref eigenvectors);
        Quaternion rotation = ComputeRotation(eigenvectors);

        Bounds rotatedBounds = ProjectVertices(vertices, centroid, rotation);
        UpdateOBB(rotatedBounds, centroid, rotation);
    }

    void UpdateOBB(Bounds rotatedBounds, Vector3 centroid, Quaternion rotation)
    {
        if (obb == null || boxCollider == null)
        {
            Debug.LogWarning("OBB references missing, reinitializing");
            InitializeOBB();
        }
        try
        {
            obb.transform.localPosition = centroid + rotation * rotatedBounds.center;
            obb.transform.localRotation = rotation;
            boxCollider.center = Vector3.zero;
            boxCollider.size = rotatedBounds.size;
        }
        catch (System.NullReferenceException)
        {
            // Handle edge cases where references become null unexpectedly
            InitializeOBB();
            UpdateOBB(rotatedBounds, centroid, rotation); // Retry
        }
    }


    bool LocalTransformsChanged()
    {
        MeshFilter[] filters = GetComponentsInChildren<MeshFilter>();

        if (prevLocalPositions == null || prevLocalRotations == null ||
            filters.Length != prevLocalPositions.Length)
        {
            return true;
        }

        for (int i = 0; i < filters.Length; i++)
        {
            Transform t = filters[i].transform;
            if (t.localPosition != prevLocalPositions[i] ||
                t.localRotation != prevLocalRotations[i])
            {
                return true;
            }
        }
        return false;
    }

    //Changed to Local Vertices because we only wanna calculate during change of child transform position.
    private List<Vector3> CollectLocalVertices(MeshFilter[] meshFilters)
    {
        vertices.Clear();

        foreach (MeshFilter mf in meshFilters)
        {
            foreach (Vector3 v in mf.sharedMesh.vertices)
            {
                // Convert to parent-relative space
                Vector3 localPos = transform.InverseTransformPoint(
                    mf.transform.TransformPoint(v)
                );
                vertices.Add(localPos);
            }
        }
        return vertices;
    }

    void StoreCurrentTransforms()
    {
        MeshFilter[] filters = GetComponentsInChildren<MeshFilter>();
        prevLocalPositions = new Vector3[filters.Length];
        prevLocalRotations = new Quaternion[filters.Length];

        for (int i = 0; i < filters.Length; i++)
        {
            Transform t = filters[i].transform;
            prevLocalPositions[i] = t.localPosition;
            prevLocalRotations[i] = t.localRotation;
        }
    }

    private Vector3 ComputeCentroid(List<Vector3> vertices)
    {
        Vector3 centroid = Vector3.zero;
        foreach (Vector3 v in vertices) centroid += v;
        return centroid / vertices.Count;
    }

    private Matrix3x3 ComputeCovarianceMatrix(List<Vector3> vertices, Vector3 centroid)
    {
        Matrix3x3 covariance = new Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);
        foreach (Vector3 v in vertices)
        {
            Vector3 d = v - centroid;
            covariance[0, 0] += d.x * d.x;
            covariance[0, 1] += d.x * d.y;
            covariance[0, 2] += d.x * d.z;
            covariance[1, 0] += d.y * d.x;
            covariance[1, 1] += d.y * d.y;
            covariance[1, 2] += d.y * d.z;
            covariance[2, 0] += d.z * d.x;
            covariance[2, 1] += d.z * d.y;
            covariance[2, 2] += d.z * d.z;
        }
        return covariance * (1.0f / (vertices.Count - 1));
    }

    private Quaternion ComputeRotation(Matrix3x3 eigenvectors)
    {
        Vector3 right = eigenvectors.GetColumn(0).normalized;
        Vector3 up = eigenvectors.GetColumn(1).normalized;
        Vector3 forward = Vector3.Cross(right, up).normalized;

        Matrix4x4 rotationMatrix = Matrix4x4.identity;
        rotationMatrix.SetColumn(0, right);
        rotationMatrix.SetColumn(1, up);
        rotationMatrix.SetColumn(2, forward);
        return rotationMatrix.rotation;
    }

    private Bounds ProjectVertices(List<Vector3> vertices, Vector3 centroid, Quaternion rotation)
    {
        Bounds bounds = new Bounds();
        foreach (Vector3 v in vertices)
        {
            Vector3 local = Quaternion.Inverse(rotation) * (v - centroid);
            bounds.Encapsulate(local);
        }
        return bounds;
    }

    // Eigenvalue decomposition using Jacobi method
    private void Jacobi(Matrix3x3 a, out Matrix3x3 v, out Vector3 eigenvalues)
    {
        v = Matrix3x3.identity;
        eigenvalues = new Vector3(a[0, 0], a[1, 1], a[2, 2]);

        for (int iter = 0; iter < 50; iter++)
        {
            float maxOffDiag = MaxOffDiagonal(a);
            if (maxOffDiag < 1e-6f) break;

            for (int p = 0; p < 2; p++)
            {
                for (int q = p + 1; q < 3; q++)
                {
                    float apq = a[p, q];
                    float app = a[p, p];
                    float aqq = a[q, q];

                    float theta = 0.5f * Mathf.Atan2(2 * apq, aqq - app);
                    float c = Mathf.Cos(theta);
                    float s = Mathf.Sin(theta);

                    // Update matrix a
                    float newApp = c * c * app - 2 * s * c * apq + s * s * aqq;
                    float newAqq = s * s * app + 2 * s * c * apq + c * c * aqq;
                    a[p, p] = newApp;
                    a[q, q] = newAqq;
                    a[p, q] = a[q, p] = 0;

                    // Update eigenvectors
                    for (int i = 0; i < 3; i++)
                    {
                        float vip = v[i, p];
                        float viq = v[i, q];
                        v[i, p] = c * vip - s * viq;
                        v[i, q] = s * vip + c * viq;
                    }

                    // Update remaining elements
                    for (int r = 0; r < 3; r++)
                    {
                        if (r != p && r != q)
                        {
                            float arp = a[r, p];
                            float arq = a[r, q];
                            a[r, p] = c * arp - s * arq;
                            a[p, r] = a[r, p];
                            a[r, q] = s * arp + c * arq;
                            a[q, r] = a[r, q];
                        }
                    }
                }
            }
        }

        eigenvalues = new Vector3(a[0, 0], a[1, 1], a[2, 2]);
    }

    private float MaxOffDiagonal(Matrix3x3 a)
    {
        return Mathf.Max(
            Mathf.Max(Mathf.Abs(a[0, 1]), Mathf.Abs(a[0, 2])),
            Mathf.Abs(a[1, 2])
        );
    }

    //bubble sort
    private void SortEigenvectors(ref Vector3 eigenvalues, ref Matrix3x3 eigenvectors)
    {
        for (int i = 0; i < 3; i++)
        {
            for (int j = i + 1; j < 3; j++)
            {
                if (eigenvalues[j] > eigenvalues[i])
                {
                    // Swap eigenvalues
                    float tempEigen = eigenvalues[i];
                    eigenvalues[i] = eigenvalues[j];
                    eigenvalues[j] = tempEigen;

                    // Swap eigenvectors
                    for (int k = 0; k < 3; k++)
                    {
                        float tempVec = eigenvectors[k, i];
                        eigenvectors[k, i] = eigenvectors[k, j];
                        eigenvectors[k, j] = tempVec;
                    }
                }
            }
        }
    }

    private void OnDrawGizmos()
    {
        if (boxCollider != null)
        {
            Gizmos.color = Color.cyan;
            Gizmos.matrix = obb.transform.localToWorldMatrix;
            Gizmos.DrawWireCube(boxCollider.center, boxCollider.size);
        }
    }

}

public struct Matrix3x3
{
    private float[,] m;

    public Matrix3x3(float m00, float m01, float m02,
                     float m10, float m11, float m12,
                     float m20, float m21, float m22)
    {
        // Initialize array with proper dimensions
        m = new float[3, 3];
        m[0, 0] = m00; m[0, 1] = m01; m[0, 2] = m02;
        m[1, 0] = m10; m[1, 1] = m11; m[1, 2] = m12;
        m[2, 0] = m20; m[2, 1] = m21; m[2, 2] = m22;
    }

    public float this[int row, int col]
    {
        get => m[row, col];
        set => m[row, col] = value;
    }

    public static Matrix3x3 operator *(Matrix3x3 a, float scalar)
    {
        // Initialize with explicit constructor
        Matrix3x3 result = new Matrix3x3(
            a[0, 0] * scalar, a[0, 1] * scalar, a[0, 2] * scalar,
            a[1, 0] * scalar, a[1, 1] * scalar, a[1, 2] * scalar,
            a[2, 0] * scalar, a[2, 1] * scalar, a[2, 2] * scalar
        );
        return result;
    }

    public Vector3 GetColumn(int index)
    {
        return new Vector3(m[0, index], m[1, index], m[2, index]);
    }

    public void SetColumn(int index, Vector3 column)
    {
        m[0, index] = column.x;
        m[1, index] = column.y;
        m[2, index] = column.z;
    }

    public static Matrix3x3 identity => new Matrix3x3(
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    );
}
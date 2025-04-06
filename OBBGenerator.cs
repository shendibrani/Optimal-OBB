using UnityEngine;
using System.Collections.Generic;
using System.Diagnostics;

#if UNITY_EDITOR
using UnityEditor;
#endif

//Attach to the parent GameObject containing MeshFilters in its children.
[ExecuteAlways]
public class OBBGenerator : MonoBehaviour
{
    public bool RuntimeUpdates = true;
    public float UpdateInterval = 0.25f;
    public PointCollectionMethod PointMethod = PointCollectionMethod.BoundingBoxCorners;

    public enum PointCollectionMethod
    {
        [Tooltip("Faster: Uses 8 corners of each child mesh's bounding box. Good approximation.")]
        BoundingBoxCorners,
        [Tooltip("Slower, Potentially Tighter Fit: Uses all vertices from child meshes.")]
        AllVertices
    }

    [Header("References")]
    [SerializeField, Tooltip("The generated OBB GameObject (auto-created if null).")]
    private GameObject obb;
    [SerializeField, Tooltip("The BoxCollider component on the OBB GameObject.")]
    private BoxCollider boxCollider;

    // Internal state
    private Vector3[] prevLocalPositions;
    private Quaternion[] prevLocalRotations;
    private Vector3[] prevLocalScales; // Added scale checking
    private float updateTimer;
    private Stopwatch swatch = new Stopwatch(); // Consider removing if not used for profiling output

    // Use a reusable list to reduce garbage collection
    private List<Vector3> pointsForOBB = new List<Vector3>();
    private Vector3[] localBoundsCorners = new Vector3[8]; // Reusable array for corner calculation

    void Start()
    {
        // Initialize in Start for runtime
        if (Application.isPlaying)
        {
            InitializeOBB();
            // Force initial calculation and state storage
            if (LocalTransformsChanged()) // Check if initial state needs storing
            {
                GenerateOBB();
                StoreCurrentTransforms();
            }
        }
    }


    void OnEnable()
    {
        EnsureOBBReference();
#if UNITY_EDITOR
        EditorApplication.update += EditorUpdate;
        if (!Application.isPlaying && (obb == null || LocalTransformsChanged()))
        {
            GenerateOBB(); 
            StoreCurrentTransforms();
        }
#endif
    }

    void OnDisable()
    {
#if UNITY_EDITOR
        EditorApplication.update -= EditorUpdate; 
#endif
    }

    void Update()
    {
        // Runtime update logic
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
    // Editor update logic
    void EditorUpdate()
    {
        if (!Application.isPlaying)
        {
            // Ensure reference is valid even if assembly reloads happen
            EnsureOBBReference();
            CheckForChanges();
        }
    }
#endif

    void EnsureOBBReference()
    {
        // Try to find existing OBB if reference is lost or null
        if (obb == null)
        {
            Transform existingOBB = transform.Find("Dynamic OBB");
            if (existingOBB != null)
            {
                obb = existingOBB.gameObject;
                boxCollider = obb.GetComponent<BoxCollider>();
                // Hide generated object flags in editor
                obb.hideFlags = HideFlags.NotEditable | HideFlags.DontSaveInBuild;

            }
        }

        // Create new OBB if still missing
        if (obb == null)
        {
            InitializeOBB();
            // Force recalculation after initialization if transforms might be non-default
            GenerateOBB();
            StoreCurrentTransforms();
        }
        else if (boxCollider == null) // Ensure BoxCollider exists if OBB exists
        {
            boxCollider = obb.GetComponent<BoxCollider>();
            if (boxCollider == null)
            {
                boxCollider = obb.AddComponent<BoxCollider>();
            }
        }
    }

    void InitializeOBB()
    {
        // Check again for existing OBB before creating
        Transform existingOBB = transform.Find("Dynamic OBB");
        if (existingOBB != null)
        {
            obb = existingOBB.gameObject;
            boxCollider = obb.GetComponent<BoxCollider>();
            if (boxCollider == null)
            {
                boxCollider = obb.AddComponent<BoxCollider>();
            }
            // Hide generated object flags in editor
            obb.hideFlags = HideFlags.NotEditable | HideFlags.DontSaveInBuild;
            return; // Found existing, no need to create
        }


        // Create new OBB
        obb = new GameObject("Dynamic OBB");
        obb.transform.SetParent(transform, false); // Set parent without changing world position initially
        obb.transform.localPosition = Vector3.zero; // Reset local position/rotation
        obb.transform.localRotation = Quaternion.identity;
        obb.transform.localScale = Vector3.one; // Reset local scale

        boxCollider = obb.AddComponent<BoxCollider>();
        boxCollider.enabled = true; // Ensure collider is active

        // Hide generated object flags in editor
        obb.hideFlags = HideFlags.NotEditable | HideFlags.DontSaveInBuild;


        // Mark the current component as dirty to save the reference to the new obb
#if UNITY_EDITOR
        if (!Application.isPlaying)
        {
            // Register the creation for Undo functionality
            Undo.RegisterCreatedObjectUndo(obb, "Create Dynamic OBB");
            // Mark the parent component dirty to save the serialized fields (obb, boxCollider)
            EditorUtility.SetDirty(this);
        }
#endif
    }


    void CheckForChanges()
    {
        // Ensure OBB exists before checking transforms
        EnsureOBBReference();
        if (obb == null) return; // If OBB couldn't be ensured, exit

        if (LocalTransformsChanged())
        {
            //UnityEngine.Debug.Log("Change detected, generating OBB...");
            GenerateOBB();
            StoreCurrentTransforms();
        }
    }

    void GenerateOBB()
    {
        EnsureOBBReference(); // Make sure references are valid
        if (obb == null || boxCollider == null)
        {
            UnityEngine.Debug.LogError("OBB or BoxCollider reference is null after EnsureOBBReference. Cannot generate OBB.", this);
            return;
        }


        swatch.Restart();

        MeshFilter[] meshFilters = GetComponentsInChildren<MeshFilter>(false);

        // Filter out mesh filters that are children of the OBB itself or have no mesh
        List<MeshFilter> validFilters = new List<MeshFilter>();
        foreach (var mf in meshFilters)
        {
            if (mf.transform == obb.transform || mf.sharedMesh == null || !mf.gameObject.activeInHierarchy)
                continue;
            // Ensure the filter is not a child of the OBB object itself
            if (mf.transform.IsChildOf(obb.transform))
                continue;

            validFilters.Add(mf);
        }


        // Two collection methods for optimization
        if (PointMethod == PointCollectionMethod.BoundingBoxCorners)
        {
            CollectBoundsCorners(validFilters, pointsForOBB);
        }
        else
        {
            CollectAllVertices(validFilters, pointsForOBB);
        }

        swatch.Stop();
        UnityEngine.Debug.Log($"Point Collection ({PointMethod}): {swatch.Elapsed.TotalMilliseconds} ms for {pointsForOBB.Count} points.");


        if (pointsForOBB.Count < 3) // Need at least 3 points to define a plane, more for a volume
        {
            // UnityEngine.Debug.LogWarning("Not enough valid points found to generate OBB. Resetting OBB.", this);
            // Reset OBB to default state if no points found
            obb.transform.localPosition = Vector3.zero;
            obb.transform.localRotation = Quaternion.identity;
            boxCollider.center = Vector3.zero;
            boxCollider.size = Vector3.zero;
            return; // Exit if no valid points
        }


        // --- OBB Calculation (PCA) ---
        swatch.Restart();

        Vector3 centroid = ComputeCentroid(pointsForOBB);
        Matrix3x3 covariance = ComputeCovarianceMatrix(pointsForOBB, centroid);

        Matrix3x3 eigenvectors = Matrix3x3.identity;
        Vector3 eigenvalues = Vector3.zero; // Eigenvalues are not directly used for the box, but calculated by Jacobi
        Jacobi(covariance, out eigenvectors, out eigenvalues); // Find principal axes

        SortEigenvectors(ref eigenvalues, ref eigenvectors);
        Quaternion rotation = ComputeRotation(eigenvectors);

        Bounds rotatedBounds = ProjectPoints(pointsForOBB, centroid, rotation);
        UpdateOBB(rotatedBounds, centroid, rotation);

        swatch.Stop();
        UnityEngine.Debug.Log($"OBB Calculation: {swatch.Elapsed.TotalMilliseconds} ms");
    }

    void UpdateOBB(Bounds rotatedBounds, Vector3 centroid, Quaternion rotation)
    {
        if (obb == null || boxCollider == null)
        {
            UnityEngine.Debug.LogWarning("OBB references became null during update, reinitializing.", this);
            InitializeOBB();
            return;
        }

        // Set the OBB transform relative to the parent
        obb.transform.localPosition = centroid; // Centroid is already in parent's local space
        obb.transform.localRotation = rotation;  // Rotation is relative to parent's local space axes

        // The bounds were calculated in the rotated frame centered at the centroid.
        // We need to adjust the OBB's position slightly to account for the center of these bounds,
        // and then set the BoxCollider's center to zero relative to the OBB's final transform.
        obb.transform.localPosition = centroid + rotation * rotatedBounds.center;

        // BoxCollider center is relative to the OBB's transform
        boxCollider.center = Vector3.zero; // Center is handled by the OBB's position offset
        boxCollider.size = rotatedBounds.size; // Size is correct in the rotated frame

        // Ensure the OBB itself doesn't affect physics/rendering if it's just a calculation helper
        // boxCollider.enabled = true; // Or false, depending on whether you need the collider active
    }


    bool LocalTransformsChanged()
    {
        // Get current MeshFilters, excluding inactive ones and the OBB itself
        MeshFilter[] currentFilters = GetComponentsInChildren<MeshFilter>(false);
        List<MeshFilter> validFilters = new List<MeshFilter>();
        if (obb != null) // Ensure obb ref is valid before filtering
        {
            foreach (var mf in currentFilters)
            {
                if (mf.transform == obb.transform || !mf.gameObject.activeInHierarchy || mf.sharedMesh == null)
                    continue;
                if (mf.transform.IsChildOf(obb.transform))
                    continue;
                validFilters.Add(mf);
            }
        }
        else
        {
            // If obb is null, consider all active filters with meshes
            foreach (var mf in currentFilters)
            {
                if (!mf.gameObject.activeInHierarchy || mf.sharedMesh == null)
                    continue;
                validFilters.Add(mf);
            }
        }


        // Check if the number of relevant filters has changed
        if (prevLocalPositions == null || prevLocalRotations == null || prevLocalScales == null ||
            validFilters.Count != prevLocalPositions.Length)
        {
            //UnityEngine.Debug.Log("Change detected: Filter count mismatch.");
            return true;
        }

        // Check if any transform properties have changed for the relevant filters
        for (int i = 0; i < validFilters.Count; i++)
        {
            Transform t = validFilters[i].transform;
            if (t.localPosition != prevLocalPositions[i] ||
                t.localRotation != prevLocalRotations[i] ||
                t.localScale != prevLocalScales[i]) // Also check scale!
            {
                //UnityEngine.Debug.Log($"Change detected: Transform mismatch for {validFilters[i].name}. Pos: {t.localPosition} vs {prevLocalPositions[i]}, Rot: {t.localRotation.eulerAngles} vs {prevLocalRotations[i].eulerAngles}, Scale: {t.localScale} vs {prevLocalScales[i]}");
                return true;
            }
        }
        return false; // No changes detected
    }

    //  Collect only transformed bounding box corners
    private void CollectBoundsCorners(List<MeshFilter> meshFilters, List<Vector3> outputPoints)
    {
        outputPoints.Clear();
        Matrix4x4 parentInvMatrix = transform.worldToLocalMatrix;

        foreach (MeshFilter mf in meshFilters)
        {
            if (mf == null || mf.sharedMesh == null) continue; // Skip if filter or mesh is missing

            Bounds meshBounds = mf.sharedMesh.bounds;
            Vector3 center = meshBounds.center;
            Vector3 extents = meshBounds.extents;

            // Calculate 8 local corners relative to the mesh pivot
            localBoundsCorners[0] = center + new Vector3(-extents.x, -extents.y, -extents.z);
            localBoundsCorners[1] = center + new Vector3(extents.x, -extents.y, -extents.z);
            localBoundsCorners[2] = center + new Vector3(extents.x, -extents.y, extents.z);
            localBoundsCorners[3] = center + new Vector3(-extents.x, -extents.y, extents.z);
            localBoundsCorners[4] = center + new Vector3(-extents.x, extents.y, -extents.z);
            localBoundsCorners[5] = center + new Vector3(extents.x, extents.y, -extents.z);
            localBoundsCorners[6] = center + new Vector3(extents.x, extents.y, extents.z);
            localBoundsCorners[7] = center + new Vector3(-extents.x, extents.y, extents.z);

            //caching
            Transform mfTransform = mf.transform;
            Matrix4x4 childLocalToWorld = mfTransform.localToWorldMatrix;
            // Transform corners to parent's local space
            for (int i = 0; i < 8; i++)
            {
                Vector3 worldPos = childLocalToWorld.MultiplyPoint3x4(localBoundsCorners[i]);
                Vector3 parentLocalPos = parentInvMatrix.MultiplyPoint3x4(worldPos);
                outputPoints.Add(parentLocalPos);
            }
        }
    }

    // slower but better
    private void CollectAllVertices(List<MeshFilter> meshFilters, List<Vector3> outputPoints)
    {
        outputPoints.Clear();
        Matrix4x4 parentInvMatrix = transform.worldToLocalMatrix; // Get the matrix ONCE before the loop

        foreach (MeshFilter mf in meshFilters)
        {
            if (mf == null || mf.sharedMesh == null) continue; // Skip if filter or mesh is missing

            Vector3[] meshVertices = mf.sharedMesh.vertices;
            int vertexCount = meshVertices.Length;
            Transform mfTransform = mf.transform;
            Matrix4x4 childLocalToWorld = mfTransform.localToWorldMatrix; // Cache child matrix

            for (int i = 0; i < vertexCount; i++)
            {
                // Transform vertex: child local -> world -> parent local
                Vector3 worldPos = childLocalToWorld.MultiplyPoint3x4(meshVertices[i]);
                Vector3 parentLocalPos = parentInvMatrix.MultiplyPoint3x4(worldPos);
                outputPoints.Add(parentLocalPos);
            }
        }
    }

    // bake it up
    void StoreCurrentTransforms()
    {
        // Get current MeshFilters, excluding inactive ones and the OBB itself
        MeshFilter[] currentFilters = GetComponentsInChildren<MeshFilter>(false);
        List<MeshFilter> validFilters = new List<MeshFilter>();
        if (obb != null) // Ensure obb ref is valid before filtering
        {
            foreach (var mf in currentFilters)
            {
                if (mf.transform == obb.transform || !mf.gameObject.activeInHierarchy || mf.sharedMesh == null)
                    continue;
                if (mf.transform.IsChildOf(obb.transform))
                    continue;
                validFilters.Add(mf);
            }
        }
        else
        {
            // If obb is null, consider all active filters with meshes
            foreach (var mf in currentFilters)
            {
                if (!mf.gameObject.activeInHierarchy || mf.sharedMesh == null)
                    continue;
                validFilters.Add(mf);
            }
        }

        prevLocalPositions = new Vector3[validFilters.Count];
        prevLocalRotations = new Quaternion[validFilters.Count];
        prevLocalScales = new Vector3[validFilters.Count]; // Store scale too

        for (int i = 0; i < validFilters.Count; i++)
        {
            Transform t = validFilters[i].transform;
            prevLocalPositions[i] = t.localPosition;
            prevLocalRotations[i] = t.localRotation;
            prevLocalScales[i] = t.localScale; // Store scale
        }
        // UnityEngine.Debug.Log($"Stored transforms for {validFilters.Count} filters.");
    }


    // --- Calculation Helpers ---

    private Vector3 ComputeCentroid(List<Vector3> points)
    {
        if (points.Count == 0) return Vector3.zero;

        Vector3 centroid = Vector3.zero;

        for (int i = 0; i < points.Count; i++)
        {
            centroid += points[i];
        }
        return centroid / points.Count;
    }

    private Matrix3x3 ComputeCovarianceMatrix(List<Vector3> points, Vector3 centroid)
    {
        Matrix3x3 covariance = new Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);
        int pointCount = points.Count;
        if (pointCount < 2) return covariance; // Covariance requires at least 2 points

        for (int i = 0; i < pointCount; i++)
        {
            Vector3 p = points[i]; // Cache point access
            Vector3 d = p - centroid;
            covariance[0, 0] += d.x * d.x;
            covariance[0, 1] += d.x * d.y; // entry [0,1]
            covariance[0, 2] += d.x * d.z; // entry [0,2]
            // covariance[1, 0] += d.y * d.x; // Symmetric, same as [0,1]
            covariance[1, 1] += d.y * d.y; // entry [1,1]
            covariance[1, 2] += d.y * d.z; // entry [1,2]
            // covariance[2, 0] += d.z * d.x; // Symmetric, same as [0,2]
            // covariance[2, 1] += d.z * d.y; // Symmetric, same as [1,2] (AI suggested)
            covariance[2, 2] += d.z * d.z; // entry [2,2]
        }

        // Assign symmetric parts // AI Optimization!!
        covariance[1, 0] = covariance[0, 1];
        covariance[2, 0] = covariance[0, 2];
        covariance[2, 1] = covariance[1, 2];


        // Normalize by N or N-1 (using N-1 for sample covariance)
        float normalizationFactor = 1.0f / (pointCount - 1);
        return covariance * normalizationFactor;
    }

    private Quaternion ComputeRotation(Matrix3x3 eigenvectors)
    {
        // Ensure the columns form a right-handed orthonormal basis
        Vector3 right = eigenvectors.GetColumn(0).normalized;
        Vector3 up = eigenvectors.GetColumn(1).normalized;
        // Ensure orthogonality if Jacobi produced slightly non-orthogonal vectors due to float precision
        Vector3 forward = Vector3.Cross(right, up).normalized;

        // Check for degenerate cases (e.g., points are co-linear or co-planar)
        if (right.sqrMagnitude < 0.0001f || up.sqrMagnitude < 0.0001f || forward.sqrMagnitude < 0.0001f ||
           float.IsNaN(right.x) || float.IsNaN(up.x) || float.IsNaN(forward.x))
        {
            // Fallback to identity or axis-aligned if eigenvectors are invalid
            //UnityEngine.Debug.LogWarning("Degenerate eigenvectors found, falling back to identity rotation.");
            return Quaternion.identity;
        }

        return Quaternion.LookRotation(forward, up);
    }

    private Bounds ProjectPoints(List<Vector3> points, Vector3 centroid, Quaternion rotation)
    {
        if (points.Count == 0) return new Bounds(Vector3.zero, Vector3.zero);


        // Inverse rotation aligns points with the OBB's local axes
        Quaternion invRotation = Quaternion.Inverse(rotation);
        // Initialize bounds with the first point transformed into the OBB's local space
        Vector3 firstPointLocal = invRotation * (points[0] - centroid);
        Bounds bounds = new Bounds(firstPointLocal, Vector3.zero); // Start with zero size at the first point

        // Encapsulate the rest of the points
        for (int i = 1; i < points.Count; i++) // Start from the second point
        {
            Vector3 localPoint = invRotation * (points[i] - centroid);
            bounds.Encapsulate(localPoint);
        }
        return bounds;
    }


    //Jacobi Eigenvalue Decomposition (Thank you Google )
    private void Jacobi(Matrix3x3 a, out Matrix3x3 v, out Vector3 eigenvalues)
    {
        v = Matrix3x3.identity;
        // Use initial diagonal elements as starting guess for eigenvalues
        eigenvalues = new Vector3(a[0, 0], a[1, 1], a[2, 2]);
        Vector3 b = new Vector3(a[0, 0], a[1, 1], a[2, 2]); // Store initial diagonal
        Vector3 z = Vector3.zero; // Used to clear rotations count per sweep

        const int maxSweeps = 50; // Maximum iteration sweeps
        const float epsilon = 1e-10f; // Convergence threshold

        for (int sweep = 0; sweep < maxSweeps; sweep++)
        {
            float sumOffDiagonal = Mathf.Abs(a[0, 1]) + Mathf.Abs(a[0, 2]) + Mathf.Abs(a[1, 2]);

            // Check for convergence
            if (sumOffDiagonal < epsilon)
            {
                eigenvalues = new Vector3(a[0, 0], a[1, 1], a[2, 2]);
                return;
            }

            // Rotation threshold - only rotate if off-diagonal element is large enough
            float threshold = (sweep < 3) ? (0.2f * sumOffDiagonal / 9.0f) : 0.0f;


            // Iterate through off-diagonal elements (p, q)
            for (int p = 0; p < 2; p++)
            {
                for (int q = p + 1; q < 3; q++)
                {
                    float apq = a[p, q];
                    float absApq = Mathf.Abs(apq);

                    if (sweep > 3 && absApq < epsilon * (Mathf.Abs(eigenvalues[p]) + Mathf.Abs(eigenvalues[q])))
                    {
                        a[p, q] = a[q, p] = 0.0f; // Set small values to zero
                    }
                    else if (absApq > threshold)
                    {
                        float app = eigenvalues[p];
                        float aqq = eigenvalues[q];
                        float diff = aqq - app;
                        float t;

                        if (absApq < Mathf.Abs(diff) * 1.0e-36f) // Avoid division by near zero if diff is large
                        {
                            t = apq / diff;
                        }
                        else
                        {
                            float theta = 0.5f * diff / apq;
                            // Use stable formula for t = tan(phi)
                            t = 1.0f / (Mathf.Abs(theta) + Mathf.Sqrt(theta * theta + 1.0f));
                            if (theta < 0.0f) t = -t;
                        }


                        float c = 1.0f / Mathf.Sqrt(t * t + 1.0f); // cos(phi)
                        float s = t * c;                           // sin(phi)
                        float tau = s / (1.0f + c);
                        float h = t * apq;

                        // Update eigenvalues
                        eigenvalues[p] -= h;
                        eigenvalues[q] += h;

                        // Update matrix 'a' (diagonal elements handled by eigenvalues update)
                        a[p, q] = a[q, p] = 0.0f;

                        // Update remaining off-diagonal elements involving p and q
                        for (int r = 0; r < 3; r++)
                        {
                            if (r != p && r != q)
                            { // Update a[r, p] and a[r, q]
                                float arp = a[r, p];
                                float arq = a[r, q];
                                a[r, p] -= s * (arq + tau * arp);
                                a[r, q] += s * (arp - tau * arq);
                                // Keep matrix symmetric
                                a[p, r] = a[r, p];
                                a[q, r] = a[r, q];
                            }
                        }


                        // Update eigenvector matrix 'v'
                        for (int r = 0; r < 3; r++)
                        {
                            float vrp = v[r, p];
                            float vrq = v[r, q];
                            v[r, p] -= s * (vrq + tau * vrp);
                            v[r, q] += s * (vrp - tau * vrq);
                        }
                        z[p] += h; // Accumulate rotation magnitude for eigenvalue tracking (not strictly needed here)
                        z[q] += h;
                    }
                }
            }
            // Update eigenvalues from accumulated rotations for next sweep's threshold calculation
            b += z;
            eigenvalues = b;
            z = Vector3.zero; // Clear accumulated rotations for the next sweep
        }

        // If it didn't converge after max sweeps
        // UnityEngine.LogWarning($"Jacobi method did not converge after {maxSweeps} sweeps.");
        // Final eigenvalues are the diagonal elements of the transformed 'a'
        eigenvalues = new Vector3(a[0, 0], a[1, 1], a[2, 2]);

    }


    // Sort eigenvalues in descending order and corresponding eigenvectors
    private void SortEigenvectors(ref Vector3 eigenvalues, ref Matrix3x3 eigenvectors)
    {
        // Simple bubble sort is fine for 3 elements
        for (int i = 0; i < 2; i++)
        {
            for (int j = i + 1; j < 3; j++)
            {
                // Sort descending by eigenvalue magnitude
                if (Mathf.Abs(eigenvalues[j]) > Mathf.Abs(eigenvalues[i]))
                {
                    // Swap eigenvalues
                    float tempEigen = eigenvalues[i];
                    eigenvalues[i] = eigenvalues[j];
                    eigenvalues[j] = tempEigen;

                    // Swap corresponding eigenvector columns
                    Vector3 colI = eigenvectors.GetColumn(i);
                    Vector3 colJ = eigenvectors.GetColumn(j);
                    eigenvectors.SetColumn(i, colJ);
                    eigenvectors.SetColumn(j, colI);
                }
            }
        }
    }


    private void OnDrawGizmos()
    {
        if (boxCollider != null && obb != null && obb.activeInHierarchy)
        {
            // Use the OBB's world transform for the gizmo matrix
            Gizmos.color = Color.cyan;
            Gizmos.matrix = Matrix4x4.TRS(obb.transform.position, obb.transform.rotation, obb.transform.lossyScale);
            Gizmos.DrawWireCube(boxCollider.center, boxCollider.size); // Center and size are local to the OBB
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
using UnityEngine;
using System.Collections.Generic;
#if UNITY_EDITOR
using UnityEditor;
#endif

[ExecuteInEditMode]
public class CombinedBoxCollider : MonoBehaviour
{
    private BoxCollider boxCollider;
    private Vector3[] previousChildPositions;
    private Vector3[] previousChildScales;
    private Quaternion[] previousChildRotations;

    void OnEnable()
    {
        InitializeCollider();
        CalculateCombinedBounds();
#if UNITY_EDITOR
        if (!Application.isPlaying)
        {
            EditorApplication.update += EditorUpdate;
            SaveChildTransforms();
        }
#endif
    }
    void InitializeCollider()
    {
        boxCollider = GetComponent<BoxCollider>();
        if (boxCollider == null)
        {
            boxCollider = gameObject.AddComponent<BoxCollider>();
        }
    }
    void OnDisable()
    {
#if UNITY_EDITOR
        if (!Application.isPlaying)
        {
            EditorApplication.update -= EditorUpdate;
        }
#endif
    }

#if UNITY_EDITOR
    void EditorUpdate()
    {
        if (!Application.isPlaying && TransformsChanged())
        {
            CalculateCombinedBounds();
            SaveChildTransforms();
            SceneView.RepaintAll();
        }
    }

    bool TransformsChanged()
    {
        Transform[] children = GetComponentsInChildren<Transform>();
        if (children.Length - 1 != previousChildPositions.Length) return true;

        for (int i = 0; i < children.Length - 1; i++)
        {
            if (children[i] == transform) continue;
            int index = System.Array.IndexOf(children, children[i]) - 1;
            if (children[i].position != previousChildPositions[index] ||
                children[i].localScale != previousChildScales[index] ||
                children[i].rotation != previousChildRotations[index])
            {
                return true;
            }
        }
        return false;
    }

    void SaveChildTransforms()
    {
        List<Vector3> positions = new List<Vector3>();
        List<Vector3> scales = new List<Vector3>();
        List<Quaternion> rotations = new List<Quaternion>();

        foreach (Transform child in GetComponentsInChildren<Transform>())
        {
            if (child == transform || child.name == "Dynamic OBB") continue;
            positions.Add(child.position);
            scales.Add(child.localScale);
            rotations.Add(child.rotation);
        }

        previousChildPositions = positions.ToArray();
        previousChildScales = scales.ToArray();
        previousChildRotations = rotations.ToArray();
    }
#endif

    [ContextMenu("Recalculate Bounds")]
    public void CalculateCombinedBounds()
    {
        bool hasBounds = false;
        Bounds totalBounds = new Bounds();

        // Collect bounds from all Renderers and Colliders in children
        foreach (Renderer renderer in GetComponentsInChildren<Renderer>())
        {
            if (renderer.gameObject.name == "Dynamic OBB") continue; // Skip the Dynamic OBB object
            if (!hasBounds)
            {
                totalBounds = renderer.bounds;
                hasBounds = true;
            }
            else
            {
                totalBounds.Encapsulate(renderer.bounds);
            }
        }

        foreach (Collider collider in GetComponentsInChildren<Collider>())
        {
            if (collider == boxCollider || collider.name == "Dynamic OBB") continue; // Skip the parent's collider

            if (!hasBounds)
            {
                totalBounds = collider.bounds;
                hasBounds = true;
            }
            else
            {
                totalBounds.Encapsulate(collider.bounds);
            }
        }

        if (hasBounds)
        {
            // Convert world-space bounds to parent's local space
            Vector3 localCenter = transform.InverseTransformPoint(totalBounds.center);
            Vector3 localSize = Vector3.Scale(totalBounds.size, new Vector3(1 / transform.lossyScale.x, 1 / transform.lossyScale.y, 1 / transform.lossyScale.z));

            boxCollider.center = localCenter;
            boxCollider.size = localSize;
        }
        else
        {
            Debug.LogWarning("No Renderers or Colliders found in children.");
        }

#if UNITY_EDITOR
        if (!Application.isPlaying)
        {
            // Mark scene as dirty and handle undo
            Undo.RecordObject(boxCollider, "Update Combined Bounds");
            PrefabUtility.RecordPrefabInstancePropertyModifications(boxCollider);
        }
#endif
    }

}
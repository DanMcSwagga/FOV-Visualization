using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FieldOfView : MonoBehaviour {
    public float viewRadius;
    [Range(0, 360)]
    public float viewAngle;

    public LayerMask obstacleMask;
    public LayerMask targetMask;

    public float meshResolution;

    public MeshFilter viewMeshFilter;
    Mesh viewMesh;

    [HideInInspector]
    public List<Transform> visibleTargets = new List<Transform>();

    void Start() {
        viewMesh = new Mesh();
        viewMesh.name = "View Mesh";
        viewMeshFilter.mesh = viewMesh;

        StartCoroutine(FindTargetsWithDelay(.2f));
    }

    void LateUpdate() {
        DrawFOV();
    }

    IEnumerator FindTargetsWithDelay(float delay) {
        while (true) {
            yield return new WaitForSeconds(delay);
            FindVisibleTargets();
        }
    }

    void FindVisibleTargets() {
        visibleTargets.Clear();

        Collider[] targetsInViewRadius = Physics.OverlapSphere(transform.position, viewRadius, targetMask);

        // TODO: rework for in
        foreach (Collider targetCollider in targetsInViewRadius) {
            Transform target = targetCollider.transform;
            Vector3 dirToTarget = (target.position - transform.position).normalized;

            // If target is within our viewAngle
            if (Vector3.Angle(transform.forward, dirToTarget) < viewAngle / 2) {
                float dstToTarget = Vector3.Distance(transform.position, target.position);

                // If there are NO obstacles in the way
                if (!Physics.Raycast(transform.position, dirToTarget, dstToTarget, obstacleMask)) {
                    visibleTargets.Add(target);
                }
            }
        }
    }

    void DrawFOV() {
        // Calculate view points
        int stepCount = Mathf.RoundToInt(viewAngle * meshResolution);
        float stepAngleSize = viewAngle / stepCount;
        List<Vector3> viewPoints = new List<Vector3>();

        for (int i = 0; i < stepCount; i++) {
            float angle = transform.eulerAngles.y - viewAngle / 2 + stepAngleSize * i;
            // Debug.DrawLine(transform.position, transform.position + DirectionFromAngle(angle, true) * viewRadius, Color.grey);
            ViewCastInfo viewCast = ViewCast(angle);
            viewPoints.Add(viewCast.point);
        }

        // Set triangles
        int vertexCount = viewPoints.Count + 1; // +1 for origin vertex
        Vector3[] vertices = new Vector3[vertexCount];
        int[] triangles = new int[(vertexCount - 2) * 3];

        vertices[0] = Vector3.zero;
        for (int i = 0; i < vertexCount - 1; i++) {
            vertices[i + 1] = transform.InverseTransformPoint(viewPoints[i]);

            // Out of bounds check
            if (i < vertexCount - 2) {
                triangles[i * 3] = 0;           // 1st vertex in a triangle
                triangles[i * 3 + 1] = i + 1;   // 2nd vertex
                triangles[i * 3 + 2] = i + 2;   // 3rd vertex
            }
        }

        // Assign view mesh
        viewMesh.Clear();
        viewMesh.vertices = vertices;
        viewMesh.triangles = triangles;
        viewMesh.RecalculateNormals();
    }

    ViewCastInfo ViewCast(float globalAngle) {
        Vector3 direction = DirectionFromAngle(globalAngle, true);
        RaycastHit hit;

        if (Physics.Raycast(transform.position, direction, out hit, viewRadius, obstacleMask)) {
            return new ViewCastInfo(true, hit.point, hit.distance, globalAngle);
        }
        return new ViewCastInfo(false, transform.position + direction * viewRadius, viewRadius, globalAngle);
    }

    public Vector3 DirectionFromAngle(float angleInDegrees, bool isGlobalAngle) {
        if (!isGlobalAngle) {
            angleInDegrees += transform.eulerAngles.y;
        }
        return new Vector3(Mathf.Sin(angleInDegrees * Mathf.Deg2Rad), 0, Mathf.Cos(angleInDegrees * Mathf.Deg2Rad));
    }

    public struct ViewCastInfo {
        public bool hit;
        public Vector3 point;
        public float distance;
        public float angle;

        public ViewCastInfo(bool _hit, Vector3 _point, float _distance, float _angle) {
            hit = _hit;
            point = _point;
            distance = _distance;
            angle = _angle;
        }
    }
}

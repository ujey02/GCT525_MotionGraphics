using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BezierCurve : MonoBehaviour
{
    public GameObject[] Point_Objects;
    public GameObject Airplane;
    public LineRenderer lineRenderer;

    // # of points = 100
    public Vector3[] BezierPoints;

    Vector3[] ControlPoints = new Vector3[4];

    [Range(0.0f, 1.0f)]
    public float parameter_t; // parameter

    // Update is called once per frame
    void Update()
    {
        GetControlPoints();
        DrawBezierCurve();

        //////////////////////////
        /// #14                ///
        /// Implement Update() ///
        //////////////////////////

        // Move the airplane to the point of Bezier Curve at parameter_t (use Airplane.transform.position)

        // Compute 3 orthogonal vector3 of frenet frame

        // Visualize the axes of frenet frame (use Debug.DrawLine())

        // Generate rotation matrix with basis vectors of franet frame and convert it to a quaternion (use Matrix4x4, RotToQuat())

        // Set airplane's rotation (use Airplane.transform.rotation)

        Airplane.transform.position = ComputeBezierPoint(parameter_t);
        Vector3 curPos = Airplane.transform.position;
        Vector3 first_derivative = ComputeBezierTangent(parameter_t);
        Vector3 second_derivative = ComputeBezier2ndDerivative(parameter_t);

        Vector3 B = ComputeFrenetNormal(first_derivative, second_derivative);
        Debug.DrawLine(curPos, curPos+first_derivative*20, Color.blue);
        Debug.DrawLine(curPos, curPos+second_derivative*20, Color.red);
        Debug.DrawLine(curPos, curPos+B*20, Color.green);

        Matrix4x4 rotMat = new Matrix4x4();
        rotMat.SetColumn(0, new Vector4(second_derivative.x, second_derivative.y,second_derivative.z,0));
        rotMat.SetColumn(1, new Vector4(B.x, B.y, B.z, 0));
        rotMat.SetColumn(2, new Vector4(first_derivative.x, first_derivative.y, first_derivative.z, 0));
        rotMat.SetColumn(3, new Vector4(curPos.x,curPos.y,curPos.z,1));
        Airplane.transform.rotation = RotToQuat(rotMat);
    }

    void GetControlPoints() {
        for (int i = 0; i < 4; i++)
        {
            ControlPoints[i].x = Point_Objects[i].transform.position.x;
            ControlPoints[i].y = Point_Objects[i].transform.position.y;
            ControlPoints[i].z = Point_Objects[i].transform.position.z;
        }
    }

    Vector3 ComputeBezierPoint(float t)
    {
        Vector3 b_t = Vector3.zero;

        ///////////////////////////////////////////
        /// #8                                  ///
        /// find the point on Bezier Curve at t ///
        ///////////////////////////////////////////

        b_t.x = ControlPoints[0].x * Mathf.Pow(1 - t, 3) + ControlPoints[1].x * Mathf.Pow(1 - t, 2) * 3 * t + ControlPoints[2].x * (1 - t) * 3 * Mathf.Pow(t, 2) + ControlPoints[3].x * Mathf.Pow(t, 3);
        b_t.y = ControlPoints[0].y * Mathf.Pow(1 - t, 3) + ControlPoints[1].y * Mathf.Pow(1 - t, 2) * 3 * t + ControlPoints[2].y * (1 - t) * 3 * Mathf.Pow(t, 2) + ControlPoints[3].y * Mathf.Pow(t, 3);
        b_t.z = ControlPoints[0].z * Mathf.Pow(1 - t, 3) + ControlPoints[1].z * Mathf.Pow(1 - t, 2) * 3 * t + ControlPoints[2].z * (1 - t) * 3 * Mathf.Pow(t, 2) + ControlPoints[3].z * Mathf.Pow(t, 3);


        return b_t;
    }

    void DrawBezierCurve() {

        /////////////////////////////////////////////////////////
        /// #9                                                ///
        /// Draw a Bezier curve with points in BezierPoints[] ///
        /// (use ComputeBezierPoint(), LineRenderer)          ///
        /////////////////////////////////////////////////////////
        for (int i = 0; i < 100; i++)
        {
            BezierPoints[i] = ComputeBezierPoint(i / (float)100);
            lineRenderer.positionCount = 100;
            lineRenderer.SetPosition(i, BezierPoints[i]);
        }

        
    }

    Vector3 ComputeBezierTangent(float t) {
        Vector3 tangent = Vector3.zero;

        /////////////////////////////////////////////////////
        /// #10                                           ///
        /// compute tangent vector of a Bezier Curve at t ///
        /////////////////////////////////////////////////////
        
        float L = 3;

        //float[] B_poly1 = new float[] { 1 - t, t };
        float[] B_poly2 = new float[] { Mathf.Pow(1 - t, 2), 2 * (1 - t) * t, Mathf.Pow(t, 2) };
        //float[] B_poly3 = new float[] { Mathf.Pow(1 - t, 3), 3 * Mathf.Pow(1 - t, 2) * t, 3 * (1 - t) * Mathf.Pow(t, 2), Mathf.Pow(t, 3) };
        Vector3 sum = new Vector3(0,0,0);
        for (int i =0; i<3; i++)
        {
            Vector3 deltaP = ControlPoints[i+1] - ControlPoints[i];

            sum += B_poly2[i] * deltaP;
        }

        tangent = L * sum;


        return tangent.normalized;
    }

    Vector3 ComputeBezier2ndDerivative(float t) {
        Vector3 second_d = Vector3.zero;

        /////////////////////////////////////////////////////
        /// #11                                           ///
        /// compute 2nd derivative of a Bezier Curve at t ///
        /////////////////////////////////////////////////////
        float[] B_poly1 = new float[] { 1 - t, t };

        float L = 3;

        Vector3 sum = new Vector3(0, 0, 0);
        for (int i = 0; i < 2; i++)
        {

            Vector3 delta2P = ControlPoints[i + 2] - 2 * ControlPoints[i + 1] + ControlPoints[i];
            sum += B_poly1[i] * delta2P;

        }

        second_d = L * (L - 1) * sum;
        Vector3 e1 = ComputeBezierTangent(t);
        second_d = second_d - Vector3.Dot(second_d, e1) * e1;
        return second_d.normalized;
    }

    Vector3 Vec3Cross(Vector3 a, Vector3 b)
    {
        Vector3 v = Vector3.zero;

        /////////////////////////////////////////////////////
        /// #12                                           ///
        /// compute cross product of two vectors a and b  ///
        /////////////////////////////////////////////////////
        v.x = a.y * b.z - a.z * b.y;
        v.y = -1*(a.x * b.z - a.z * b.x);
        v.z = a.x * b.y - a.y * b.x;


        return v.normalized;
    }

    Vector3 ComputeFrenetNormal(Vector3 first_derivative, Vector3 second_derivative) {
        Vector3 N = Vector3.zero;

        /////////////////////////////////////////////////////
        /// #13                                           ///
        /// compute Normal Vector of Frenet frame         ///
        /////////////////////////////////////////////////////
        N = Vec3Cross(first_derivative, second_derivative);
        return N.normalized;
    }

    public Quaternion RotToQuat(Matrix4x4 R)
    {
        Quaternion q = new Quaternion();

        /////////////////////////////////////////////////////////////
        /// #2                                                    ///
        /// copy function implemented at #2 of ComputeRotation.cs ///
        /////////////////////////////////////////////////////////////
        q.w = Mathf.Sqrt(1 + R.m00 + R.m11 + R.m22) / 2;
        q.x = (R.m21 - R.m12) / (4 * q.w);
        q.y = (R.m02 - R.m20) / (4 * q.w);
        q.z = (R.m10 - R.m01) / (4 * q.w);

        return q;
    }
}

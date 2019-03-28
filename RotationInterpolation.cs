using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public static class MatrixExtension
{
    public static Matrix4x4 Subtract(this Matrix4x4 mat, Matrix4x4 other)
    {
        // implement

        return mat;
    }
}

public class RotationInterpolation : MonoBehaviour
{
    Matrix4x4 EulerToRot(Vector3 v)
    {
        Matrix4x4 R = new Matrix4x4();
        Matrix4x4 Rx = new Matrix4x4();
        Matrix4x4 Ry = new Matrix4x4();
        Matrix4x4 Rz = new Matrix4x4();

        ///////////////////////////////////////////////////////////////////////
        /// #1                                                              ///
        /// Implement conversion from euler angles input to rotation matrix ///
        /// !!!!!Unity3D engine uses YXZ intrinsic rotation!!!!!            ///
        ///////////////////////////////////////////////////////////////////////


        float xAng = v.x * Mathf.Deg2Rad;
        float yAng = v.y * Mathf.Deg2Rad;
        float zAng = v.z * Mathf.Deg2Rad;
       
        //Rx
        Rx.m00 = Rx.m33 = 1;
        Rx.m11 = Rx.m22 = Mathf.Cos(xAng);
        Rx.m21 = Mathf.Sin(xAng);
        Rx.m12 = -Rx.m21;
        //Ry
        Ry.m11 = Ry.m33 = 1;
        Ry.m00 = Ry.m22 = Mathf.Cos(yAng);
        Ry.m02 = Mathf.Sin(yAng);
        Ry.m20 = -Ry.m02;
        //Rz
        Rz.m22 = Rz.m33 = 1;
        Rz.m00 = Rz.m11 = Mathf.Cos(zAng);
        Rz.m10 = Mathf.Sin(zAng);
        Rz.m01 = -Rz.m10;

        R = Ry * Rx * Rz;
        return R;
    }

    Quaternion RotToQuat(Matrix4x4 R)
    {
        Quaternion q = new Quaternion();

        ///////////////////////////////////////////////////////////////
        /// #2                                                      ///
        /// Implement conversion from rotation matrix to quaternion ///
        ///////////////////////////////////////////////////////////////
        q.w = Mathf.Sqrt(1 + R.m00 + R.m11 + R.m22) / 2;
        q.x = (R.m21 - R.m12) / (4 * q.w);
        q.y = (R.m02 - R.m20) / (4 * q.w);
        q.z = (R.m10 - R.m01) / (4 * q.w);


        return q;
    }

    Vector3 RotToExp(Matrix4x4 R)
    {
        float d= 0f;
        Vector3 w_unit = Vector3.zero;

        /////////////////////////////////////////////////////////////////////////////
        /// #3                                                                    ///
        /// Implement conversion from rotation matrix to angle-axis (exponential) ///
        /////////////////////////////////////////////////////////////////////////////

        float trace = R.m00 + R.m11 + R.m22;
        d = Mathf.Acos((trace - 1) / 2);
        Matrix4x4 Rt = R.transpose;
        float val = (1 / (2 * Mathf.Sin(d)));
        Matrix4x4 wHat = new Matrix4x4();

        wHat.m00 = val * (R.m00 - Rt.m00);
        wHat.m01 = val * (R.m01 - Rt.m01);
        wHat.m02 = val * (R.m02 - Rt.m02);
        wHat.m10 = val * (R.m10 - Rt.m10);
        wHat.m11 = val * (R.m11 - Rt.m11);
        wHat.m12 = val * (R.m12 - Rt.m12);
        wHat.m20 = val * (R.m20 - Rt.m20);
        wHat.m21 = val * (R.m21 - Rt.m21);
        wHat.m22 = val * (R.m22 - Rt.m22);
        wHat.m33 = 1;

        w_unit.x = wHat.m21;
        w_unit.y = wHat.m02;
        w_unit.z = wHat.m10;

        w_unit = w_unit.normalized;


        return w_unit * d;
    }

    Quaternion ExpToQuat(Vector3 w)
    {
        Quaternion q = new Quaternion();

        ///////////////////////////////////////////////////////////
        /// #4                                                  ///
        /// Implement conversion from exponential to quaternion ///
        ///////////////////////////////////////////////////////////

        float d = Mathf.Sqrt(w.x * w.x + w.y * w.y + w.z * w.z);
        Vector3 w_unit = w / d;
        q.w = Mathf.Cos(d / 2);
        q.x = Mathf.Sin(d / 2) * w_unit.x;
        q.y = Mathf.Sin(d / 2) * w_unit.y;
        q.z = Mathf.Sin(d / 2) * w_unit.z;

        return q;
    }

    public enum InterpolationMethod { EulerAngles, ExpCoord, Quat }
    public InterpolationMethod interpmethod;

    [Range(0.0f, 1.0f)]
    public float interp; // interpolation parameter

    // Input
    public Vector3 start_EulerAngles; // initial orientation in Euler Angles
    public Vector3 end_EulerAngles; // final orientation in Euler Angles

    // Euler Angles interp
    private Vector3 interp_v;
    
    // Rotation Matrix
    private Matrix4x4 m_start, m_end; 

    // Exponential Coordinate
    private Vector3 w_start, w_end; 
    private Vector3 interp_w;

    //Quaternion
    private Quaternion q_start, q_end;
    private Quaternion interp_q;

    // GUI text
    public Text TextEuler;
    public Text TextExpCoord;
    public Text TextQuat;

    // Use this for initialization
    void Start()
    {
        TextEuler.text = "EulerAngles: (0.0, 0.0, 0.0)";
        TextExpCoord.text = "Exp. Coords: (0.0, 0.0, 0.0)";
        TextQuat.text = "Quaternion: (0.0, 0.0, 0.0, 1.0)";
    }

   

    // Update is called once per frame
    void Update()
    {
        print("HI");
        //////////////////////////////////////////////////////////////////
        /// #5                                                         ///
        /// Implement linear interpolation of euler angles             ///
        //////////////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////////////
        /// #6                                                         ///
        /// Calculate interpolated rotations and apply it to the cube  ///
        /// 1. Euler Angles (use rotation matrix)                      ///
        /// 2. Exponential Coordinate                                  ///
        /// 3. Quaternion                                              ///
        ///                                                            ///
        /// use transform.rotation = ~ /                               ///
        /// use Quaternion.Slerp() for quaternion interpolation        ///
        //////////////////////////////////////////////////////////////////

        // apply
        switch (interpmethod)
        {
            case InterpolationMethod.EulerAngles:
                interp_v = new Vector3((1 - interp) * start_EulerAngles.x + interp * end_EulerAngles.x,
                    (1 - interp) * start_EulerAngles.y + interp * end_EulerAngles.y,
                    (1 - interp) * start_EulerAngles.z + interp * end_EulerAngles.z);
                print(interp_v);
                transform.rotation = RotToQuat(EulerToRot(interp_v));
                break;

            case InterpolationMethod.ExpCoord:
                m_start = EulerToRot(start_EulerAngles);
                m_end = EulerToRot(end_EulerAngles);
                w_start = RotToExp(m_start);
                w_end = RotToExp(m_end);
                interp_w = new Vector3((1 - interp) * w_start.x + interp * w_end.x,
                   (1 - interp) * w_start.y + interp * w_end.y,
                   (1 - interp) * w_start.z + interp * w_end.z);
                print(interp_w);
                transform.rotation = ExpToQuat(interp_w);
                break;

            case InterpolationMethod.Quat:
                
                m_start = EulerToRot(start_EulerAngles);
                m_end = EulerToRot(end_EulerAngles);
                q_start = RotToQuat(m_start);
                q_end = RotToQuat(m_end);
                interp_q = Quaternion.Slerp(q_start, q_end, interp);
                transform.rotation = interp_q;
                break;
        }

        //////////////////////////////////////////////////////////////////////////////
        /// #7                                                                     ///
        /// Visualize the rotation in euler angle, angle-axis (exp) and quaternion ///
        /// ex) TextEuler.text = "EulerAngles: (" + value + ")";                   /// 
        //////////////////////////////////////////////////////////////////////////////
        TextEuler.text = "EulerAngles: (" + interp_v + ")";
        TextExpCoord.text = "Exp. Coords: (" + interp_w + ")";
        TextQuat.text = "Quaternion: ("+ interp_q + ")";
    }
}
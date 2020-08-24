using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class FK_IK_RightArm_Kyle : MonoBehaviour {

	//
    public enum Options { FK, CCDIK, JacobianIK }
    public Options option;

	//
	Transform[] Ts;
	Transform EE, Target;

	//
	int totalDoF = 0;
	Joint[] joints;
	Joint endEffector;
	List<Vector3> axes = new List<Vector3>();

    //
	double[,] J; //body Jacobian matrix
	int CCDorder;
    
    // Adjoint transformation
    // This function implements V_out = Ad_g(V_in), where g = (R,p), V_out = (w_out,v_out),  V_in = (w_in, v_in)
    void Adjoint(Vector3 p, Quaternion R, Vector3 w_in, Vector3 v_in, out Vector3 w_out, out Vector3 v_out)
    {
        w_out = R * w_in;
        v_out = Vector3.Cross(p, R * w_in) + R * v_in;
    }

    //input: exponential coordinates (randian)
    //output: quaternion corresponding to the exponential coordinates
    Quaternion exp(Vector3 expCoord)
    {
        float angle = expCoord.magnitude * 180 / Mathf.PI;
        return Quaternion.AngleAxis(angle, expCoord.normalized);
    }

    //input: quaternion 
    //output: exponential coordinates (radian)    
    Vector3 log(Quaternion quat)
    {
        float angleDeg;
        Vector3 w;
        quat.ToAngleAxis(out angleDeg, out w);
        w *= angleDeg * Mathf.PI / 180; //convert to radian
        return w;
    }

	// Use this for initialization
	void Start () {
        //init
        Ts = new Transform[3];
        joints = new Joint[3];
        
        //initialize option as FK
        option = Options.FK;

        //initialize robot body parts
        Ts[0] = GameObject.Find("Right_Upper_Arm_01").transform;
		Ts[1] = GameObject.Find("Right_Forearm_01").transform;
		Ts[2] = GameObject.Find("Right_Wrist_01").transform;

        //initialize end effector as "Right_Hand_01" and target as "TargetCube"
        EE = GameObject.Find("Right_Hand_01").transform;
		Target = GameObject.Find("TargetCube").transform;

		//initialize joint axis parameters
		joints[0] = GameObject.Find("Right_Upper_Arm_01").GetComponent<Joint>();
		joints[1] = GameObject.Find("Right_Forearm_01").GetComponent<Joint>();
		joints[2] = GameObject.Find("Right_Wrist_01").GetComponent<Joint>();

        //get total degree of freedom
        foreach (Joint joint in joints) {
			totalDoF += joint.getDoF();
		}

		//add axis
		foreach (Joint joint in joints) {
            for (int index = 0; index != joint.getDoF(); ++index) {
				axes.Add(joint.getAxis(index));
			}
		} 

		//
		J = new double[6, totalDoF]; //6x7 matrix
        CCDorder = 2; //from the terminal link
    }

    // Update is called once per frame
    void Update()
    {
		if (option == Options.JacobianIK)
			IK_Numerial ();
		else if (option == Options.CCDIK)
			IK_CCD ();
		else
			FK ();
    }

	void FK()
	{		
		joints[2].manualUpdate();
	}

    //perform CCD
    void IK_CCD()
    {
        //perform cyclic coordinate descent IK
        IK_CCD_Link(CCDorder);

        //set CCDoder to the next (proximal) one
        CCDorder -= 1;
        if (CCDorder == -1) CCDorder = 2;        
    }

    //perform CCD to the link(linkIdx)
    void IK_CCD_Link(int linkIdx)
    {
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //#5
        //TODO: rotate joint to make EE approach Target
        //You consider "TargetCube" as point at CCD method(== don't have to consider rotation of target)


        Vector3 l0 = EE.transform.position - joints[linkIdx].transform.position;
        Vector3 l1 = Target.transform.position - joints[linkIdx].transform.position;
        Vector3 w =  Vector3.Normalize(Vector3.Cross(l0, l1));
        float theta = Mathf.Rad2Deg * Mathf.Atan2(Vector3.Dot(w, Vector3.Cross(l0, l1)), Vector3.Dot(l0, l1));

        //rotate joint
        joints[linkIdx].transform.Rotate(w,theta,Space.World);
        //joints[linkIdx].transform.Rotate(joints[linkIdx].transform.rotation * w, theta);
        print("w is " + w + " and theta is " + theta + " at index " + linkIdx);



        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }

    //Jacobian-based IK
    void IK_Numerial()
    {
        //Target seen from EE
        Vector3 p = EE.InverseTransformPoint(Target.position); //Target's position seen from EE
        Quaternion R = Quaternion.Inverse(EE.rotation) * Target.rotation; //Target's rotation seen from EE
        Vector3 w = log(R);

        //return if error is small enough
        if (p.magnitude < 0.001f && w.magnitude < 0.001f) return;

        //
        //otherwise, do inverse kinematics
        //
        
        double[] b = new double[6];
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //#3
        //TODO: set b[] as the desired body velocity.
        Vector3 v = new Vector3();
        if (w == new Vector3(0,0,0))
        {
            v = p;
        }
        else
        {
            v = Vector3.Cross(p, w);
        }
        //Vector3 v = p;
        b[0] = w[0]; b[1] = w[1]; b[2] = w[2]; b[3] = v[0]; b[4] = v[1]; b[5] = v[2];
        print("point is " + p);
        print("angular rotation is " + w);
        print("body velocity is " + b[0] + "," + b[1] + "," + b[2] + "," + b[3] + "," + b[4] + "," + b[5]);
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //update body Jacobian
        UpdateBodyJacobian();
                
        //solve J x = b for x
		double[] x = new double[totalDoF];

        int info; alglib.densesolverlsreport rep;
		alglib.rmatrixsolvels(J, 6, totalDoF, b, 1e-3, out info, out rep, out x);
        
        //rotate each joint by x        
		//T0.localRotation = T0.localRotation * exp(axes[0] * (float)(x[0]) + axes[1] * (float)(x[1]) + axes[2] * (float)(x[2]));
		//T1.localRotation = T1.localRotation * exp(axes[3] * (float)(x[3]));
		//T2.localRotation = T2.localRotation * exp(axes[4] * (float)(x[4]) + axes[5] * (float)(x[5]) + axes[6] * (float)(x[6]));
		//
		int totalDoFIndex = 0;
		//
		for (int transfIndex=0; transfIndex!=3; ++transfIndex) {			
			//
			Vector3 expCoord = new Vector3(0,0,0);
			for (int axisIndex = 0; axisIndex != joints[transfIndex].getDoF(); ++axisIndex) {
				expCoord += joints[transfIndex].getAxis(axisIndex) * (float)(x[totalDoFIndex++]);
			}
			//
			Ts[transfIndex].localRotation *= exp(expCoord);
		}
       
    }

    // Update Body Jacobian matrix
    void UpdateBodyJacobian()
    {
		Vector3[] Jw = new Vector3[totalDoF]; //top three rows (for rotation) of Jacobian
		Vector3[] Jv = new Vector3[totalDoF]; //bottom three rows (for translation) of Jacobian

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //#4
        //TODO: Computer Jw[] and Jv[]. 
        //you can also use ' Adjoint(`) ' for computing elements of J

        int n = 0;
        int m = 0;
        foreach (Joint joint in joints)
        {
            for (int i = 0; i <joint.getDoF(); ++i)
            {
                //print("current joint number is " + n + ", current dof number is " + m);

                Quaternion R = new Quaternion();
                Vector3 p = new Vector3();
                R = Quaternion.Inverse(EE.rotation) * joint.transform.rotation; //Joint's rotation seen from EE
                p = EE.InverseTransformPoint(joint.transform.position); // Joint position seen from EE

                Vector3 w = axes[m];
                Vector3 v = new Vector3(0,0,0);

                Vector3 w_out = new Vector3();
                Vector3 v_out = new Vector3();

                Adjoint(p, R, w, v, out w_out, out v_out);

                Jw[m] = w_out;
                Jv[m] = v_out;

                m = m + 1;
            }
            n = n + 1;

        }

       

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //Pack Jw[] and Jv[] into J
        for (int i = 0; i < totalDoF; ++i)
        {
            J[0, i] = Jw[i].x;
            J[1, i] = Jw[i].y;
            J[2, i] = Jw[i].z;
            J[3, i] = Jv[i].x;
            J[4, i] = Jv[i].y;
            J[5, i] = Jv[i].z;
        }        

    }
}

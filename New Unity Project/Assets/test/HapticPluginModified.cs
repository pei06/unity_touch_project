using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Threading;
//using System.Numerics;

using System.IO;
using System.Runtime.InteropServices;

#if UNITY_EDITOR
using UnityEditor;
#endif


//! This MonoBehavior should be attached to a GameObject and will represent the Haptic Device itself.
//! One of these objects should be added to the scene corrisponding to each Haptic Device you intend the scene to connect to. 
//! Additionally this object contains static declarations of the functions in the OHToUnityBridge dll, required for this asset to function.
public class HapticPluginModified : MonoBehaviour
{


	[Header("Configuration Attributes")]

	public string configName = "Default Device";  //!< Filename of the Haptic Device Configuration. (Typically "Default Device")
	public string remotTargetIPAdress = "192.168.50.111";
	public int remoteTargetPort = 123;
	public bool connect_On_Start = true;        //!< Should the script connect to haptic the moment it's created?

	[Range(0.0f, 1.0f)] public float PhysicsForceStrength = 0.333f; //!< Spring force coupling the haptic forces and the Unity Physics simualtion
	[Range(0.0f, 1.0f)] public float PhysicsForceDamping = 0.33f; //!< Damping force for the spring coupling the haptic forces and the Unity Physics simulation.

	public bool shapesEnabled = true; //TODO FIXME Doesn't work yet.

	public GameObject hapticManipulator = null; //!< Reference to unity gameobject representing the haptic stylus.
	public bool PhysicsManipulationEnabled = true;  //!< Should the haptic forces interact with the Unity physics simulation?


	[Header("ReadOnly Attributes")]

	[DisplayOnlyAttribute] public GameObject touching = null;  //!< (Readonly) Reference to the touchable unity gameobject currently being touched
	[DisplayOnlyAttribute] public float touchingDepth = 0; //!< (Readonly) Depth (ie: pressure) the 'touching' object is being touched.



	[DisplayOnlyAttribute] public int hHD = -1;
	[DisplayOnlyAttribute] public string device_SerialNumber = "-Not Connected-";   //!< (Readonly) Serial number of the haptic device.
	[DisplayOnlyAttribute] public string device_Model = "-Not Connected-"; //!< (Readonly) Model of the haptic device

	[DisplayOnlyAttribute] public Vector3 stylusPositionRaw;    //!< (Readonly) Stylus position, in device coordinates.
	[DisplayOnlyAttribute] public Vector3 stylusVelocityRaw;    //!< (Readonly) Stylus velocity, in device coordinates.
	[DisplayOnlyAttribute] public Matrix4x4 stylusTransformRaw; //!< (Readonly) Stylus transform, in device coordinates.
	[DisplayOnlyAttribute] public int[] Buttons;                //!< (Readonly) Array of buttons. 1 is pressed, 0 is unpressed.
	[DisplayOnlyAttribute] public int inkwell;                  //!<! (Readonly) 1 if inkwell button is triggered. 0 otherwise. (Not all devices have inkwells.)


	[DisplayOnlyAttribute] public Vector3 proxyPositionRaw;         //!< (Readonly) Proxy position, in device coordinates.
	[DisplayOnlyAttribute] public Quaternion proxyOrientationRaw;   //!< (Readonly) Proxy Orientation, in device coordinates.
	[DisplayOnlyAttribute] public Matrix4x4 proxyTransformRaw;      //!< (Readonly) Proxy Transform in device coordinates.
	[DisplayOnlyAttribute] public Vector3 proxyNormalRaw;



	#region DLL_Imports
	[DllImport("OHToUnityBridge")] public static extern void getVersionString(StringBuilder dest, int len);  //!< Retreives the OpenHaptics version string.

	// Setup Functions
	[DllImport("OHToUnityBridge")] public static extern int initDevice(string deviceName);  //!< Connects to and Initializes a haptic device.
	[DllImport("OHToUnityBridge")] public static extern void getDeviceSN(string configName, StringBuilder dest, int len);   //!< Retrieves device serial number
	[DllImport("OHToUnityBridge")] public static extern void getDeviceModel(string configName, StringBuilder dest, int len);    //!< Retrieves devices model name
	[DllImport("OHToUnityBridge")] public static extern void startSchedulers(); //!< Starts the Open Haptic schedulers and assigns the required internal callbacks

	// Device Information
	//! Retrieves the bounds created by the physical limitations of the device.
	//! Equivialant to an `hlWorkspace` call.
	[DllImport("OHToUnityBridge")] public static extern void getWorkspaceArea(string configName, double[] usable6, double[] max6);

	// Updates
	[DllImport("OHToUnityBridge")] public static extern void getPosition(string configName, double[] position3);
	[DllImport("OHToUnityBridge")] public static extern void getVelocity(string configName, double[] velocity3);
	[DllImport("OHToUnityBridge")] public static extern void getTransform(string configName, double[] matrix16);
	[DllImport("OHToUnityBridge")] public static extern void getButtons(string configName, int[] buttons4, ref int inkwell);

	// Force output
	[DllImport("OHToUnityBridge")] public static extern void setForce(string configName, double[] lateral3, double[] torque3); //!< Adds an additional force to the haptic device. Can be eseed for scripted forces, but in most cases using an Effect is preferable. 

	//! The haptic device and the unity gameobject are connected by a simulated spring. (Because Unity runs at a much slower framerate than is required for directly setting physics forces.)
	//! This call sets the current anchor position of the spring.  Normally the HapticPlugin script would call this during its update.
	[DllImport("OHToUnityBridge")] public static extern void setSpringAnchorPosition(string configName, double[] position3, double[] velocity3);
	//! The haptic device and the unity gameobject are connected by a simulated spring. (Because Unity runs at a much slower framerate than is required for directly setting physics forces.)
	//! This call sets the current stiffness of the spring. 
	[DllImport("OHToUnityBridge")] public static extern void setSpringStiffness(string configName, double stiffness, double damping);

	// Shape Handling
	//! Allocates an OH touchable shape. Typically called by the HapticPlugin object.
	//! \return the handle ID for the newly created shape.
	[DllImport("OHToUnityBridge")] public static extern void shape_define(int id, string name, double[] ParticleSystemVertexStreams, int[] triangles, int vertCount, int triCount);
	[DllImport("OHToUnityBridge")] public static extern void shape_setTransform(int id, double[] matrix16); //!< Sets the transform of an already defined touchable shape.  Typically called by HapticPlugin update function.
	[DllImport("OHToUnityBridge")] public static extern void shape_remove(int id);  //!< Removes an already defined touchable shape.
	[DllImport("OHToUnityBridge")] public static extern void shape_removeAll(); //!< Removes **all** touchable shapes.
																				//! If the Proxy Stylus is currently touching a touchable object, this will retrieve the ID and the current depth (pressure) of the stylus.
																				//! \return true if currently touching an object.
	[DllImport("OHToUnityBridge")] public static extern bool shape_getTouched(string configName, ref int shapeID, ref double depth);
	//! Sets the parameters of a touchable object.
	//! Refer to OH documentation for the meaning of the parameters.  
	//! See also the helper script HapticSurface
	[DllImport("OHToUnityBridge")] public static extern void shape_settings(int id, double hlStiffness, double hlDamping, double hlStaticFriction, double hlDynamicFriction, double hlPopThrough);

	//! Sets the current constraint parameters of a touchable object.
	//! See also the helper script HapticSurface
	[DllImport("OHToUnityBridge")] public static extern void shape_constraintSettings(int id, int model, double snapDist);
	//! If an object's normals are backwards, it can be corrected with this function.
	[DllImport("OHToUnityBridge")] public static extern void shape_flipNormals(int id, bool flipNormals);
	//! Define which surface of the object (Inner or Outer) should be touchable.
	//! \param facing  1=HL_FRONT, 2=HL_BACK, 3=HL_FRONT_AND_BACK
	[DllImport("OHToUnityBridge")] public static extern void shape_facing(int id, int facing); //1=HL_FRONT, 2=HL_BACK, 3=HL_FRONT_AND_BACK


	//! Called every update frame, **after** all the shape positions and parameters have been updated.
	[DllImport("OHToUnityBridge")] public static extern void shape_render(string configName, double[] matrix16);

	[DllImport("OHToUnityBridge")] public static extern void shape_enableShapeRendering();
	[DllImport("OHToUnityBridge")] public static extern void shape_disableShapeRendering();


	[DllImport("OHToUnityBridge")] public static extern void getProxyPosition(string configName, double[] position3);
	[DllImport("OHToUnityBridge")] public static extern void getProxyRotation(string configName, double[] quaternion4);
	[DllImport("OHToUnityBridge")] public static extern void getProxyTouchNormal(string configName, double[] normal3);
	[DllImport("OHToUnityBridge")] public static extern void getProxyTransform(string configName, double[] matrix16);


	[DllImport("OHToUnityBridge")] public static extern void effects_resetAll(); //!< Delete all Effects.

	//! Allocate a new OpenHaptics Effect
	//! \return The handle ID for the new effect.
	[DllImport("OHToUnityBridge")] public static extern int effects_assignEffect(string configName);
	[DllImport("OHToUnityBridge")] public static extern void effects_startEffect(string configName, int ID);
	[DllImport("OHToUnityBridge")] public static extern void effects_stopEffect(string configName, int ID);

	//! Update the parameters of the effect.
	//! (Unused parameters may be set to 0.0)
	//! \param gain	The strength of the effect. Used for **Vibrate**, **Viscocity**, **Spring**, and **Friction**
	//! \param magnitude Maximum force limit. Used for **Vibrate**, **Constant**, **Viscocity**, **Spring**, and **Friction**
	//! \param frequency Vibration frequency.  Used for **Vibrate**
	//! \param position3 A point (defined as an array of 3 doubles) defining the focal point of the effect. Used for **Spring**
	//! \param direction3 A unit vector (defined as an array of 3 doubles) defining the direction of the effect. Used for **Vibrate**, and **Constant**
	[DllImport("OHToUnityBridge")] public static extern void effects_settings(string configName, int ID, double gain, double magnitude, double frequency, double[] position3, double[] direction3);
	[DllImport("OHToUnityBridge")] public static extern void effects_deleteEffect(string configName, int ID);

	//! Assigns the *type* of the effect.
	//! \param type 0 = constant, 1 = spring, 2 = viscous, 3 = friction, 4 = vibration
	[DllImport("OHToUnityBridge")] public static extern void effects_type(string configName, int ID, int type);




	//Cleanup functions
	//! Disconnects from all devices.
	//! In the process all **shapes** and all **effects** are also cleared.
	[DllImport("OHToUnityBridge")] public static extern void disconnectAllDevices();

	//Error Handling Functions
	[DllImport("OHToUnityBridge")] public static extern int getHDError(StringBuilder Info, int len);
	[DllImport("OHToUnityBridge")] public static extern int getHLError(StringBuilder Info, int len);

	#endregion

	private Queue hapticErrorQueue;

	// Privates

	[System.NonSerialized] public double[] max_extents = new double[6];
	[System.NonSerialized] public double[] usable_extents = new double[6];

	private double scale = 1.0f;

	// "Cooked" values. Raw position data multiplied by this object's transform.
	// Updated every frame.
	Matrix4x4 stylusMatrixWorld;
	public Vector3 stylusPositionWorld;
	Vector3 stylusVelocityWorld;
	public Quaternion stylusRotationWorld;

	private GameObject[] touchableObjects;


	private bool safetyMode = false;

	private bool showNoDevicePopup = false;
	private bool showOldVersionPopup = false;

	private bool isIncorrectVersion = false;

	//new defined
	private double[] iniPosInput = new double[3];
	private IPEndPoint ipEndPoint;
	private UdpClient udpClient;
	private byte[] sendByte;
	private Vector3 incision = new Vector3(350, -60, 300);
	private Vector3 offset = new Vector3(0, -100, 100);
	private Vector3 rcm = new Vector3(350, -80, 270);
	//private Vector3 offset = new Vector3(0, 0, 0);
	private float distance = 300;
	private float toollength = 200;

	private Matrix4x4 iniOrientation;
	private Matrix4x4 screenOrientation;
	private float screenAngle;
	private float motionScalingFactor = 1.0f;

	// Use this for initialization
	void OnEnable()
	{
		Buttons = new int[4];

		// Get Version String
		StringBuilder sb = new StringBuilder(256);
		getVersionString(sb, sb.Capacity);
		Debug.Log("Haptic Plugin Version : " + sb.ToString());

		if (connect_On_Start)
		{
			initializeHapticDevice();
		}

		if (hapticManipulator != null)
		{
			Rigidbody body = hapticManipulator.GetComponent<Rigidbody>();
			if (body != null) // Put the cursor on the device so it doesn't jump.
			{
				safeUpdateManipulator();
			}
		}

		touchableObjects = GameObject.FindGameObjectsWithTag("Touchable") as GameObject[];  //FIXME  Does this fail gracefully?

		hapticErrorQueue = new Queue();

	}
	void Start()
	{
		if (isIncorrectVersion) return;


		startSchedulers();
		if (this.shapesEnabled) setupShapes();

		//modified
		getPosition(configName, iniPosInput);
		ipEndPoint = new IPEndPoint(IPAddress.Parse(remotTargetIPAdress), remoteTargetPort); // change server ip here
		udpClient = new UdpClient();
		iniOrientation.SetRow(0, new Vector4(-1, 0, 0, 0));
		iniOrientation.SetRow(1, new Vector4(0, 0, 1, 0));
		iniOrientation.SetRow(2, new Vector4(0, 1, 0, 0));
		iniOrientation.SetRow(3, new Vector4(0, 0, 0, 1));

		screenAngle = 65 * Mathf.Deg2Rad;

		screenOrientation.SetRow(0, new Vector4(1, 0, 0, 0));
		screenOrientation.SetRow(1, new Vector4(0, Mathf.Cos(screenAngle), Mathf.Sin(screenAngle), 0));
		screenOrientation.SetRow(2, new Vector4(0, -Mathf.Sin(screenAngle), Mathf.Cos(screenAngle), 0));
		screenOrientation.SetRow(3, new Vector4(0, 0, 0, 1));




	}

	void OnDestroy()
	{
		if (isIncorrectVersion) return;

		Debug.Log("Disconnecting from Haptic");
		disconnectAllDevices();
	}
	void OnDisable()
	{
		if (isIncorrectVersion) return;

		Debug.Log("OnDisable");
		double[] zero = { 0.0, 0.0, 0.0 };
		setSpringStiffness(configName, 0.0, 0.0);
		setForce(configName, zero, zero);
		shape_removeAll();
		effects_resetAll();
	}

	void OnApplicationQuit()
	{
		if (isIncorrectVersion) return;
		Debug.Log("OnApplicationQuit: Disconnecting from Haptic");


		double[] zero = { 0.0, 0.0, 0.0 };
		setSpringStiffness(configName, 0.0, 0.0);
		setForce(configName, zero, zero);
		shape_removeAll();
		effects_resetAll();

		double[] M = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		shape_render(configName, M);
	}



	bool initializeHapticDevice()
	{
		bool success = false;
		hHD = initDevice(configName);
		if (hHD < 0)
		{
			//Error. 
			device_SerialNumber = "-Not Connected-";
			device_Model = "-Not Connected-";
			success = false;
			showNoDevicePopup = true;

			if (hHD == -1001) // Constant indicating incorrect OH Version
			{
				showOldVersionPopup = true;
				isIncorrectVersion = true;
				hapticErrorQueue.Enqueue(System.DateTime.Now.ToLongTimeString() + " - " + "Incorrect Open Haptic Version.");
			}


		}
		else
		{
			{
				StringBuilder sb = new StringBuilder(256);
				getDeviceSN(configName, sb, sb.Capacity);
				device_SerialNumber = sb.ToString();
			}
			{
				StringBuilder sb = new StringBuilder(256);
				getDeviceModel(configName, sb, sb.Capacity);
				device_Model = sb.ToString();
			}

			getWorkspaceArea(configName, usable_extents, max_extents);

			success = true;
			showNoDevicePopup = false;
			isIncorrectVersion = false;
		}

		return success;
	}

	// Update is called once per frame
	void FixedUpdate()
	{
		if (isIncorrectVersion) return;
		checkErrors();

		if (hHD < 0)
			return;

		double[] M16 = MatrixToDoubleArray(this.transform.worldToLocalMatrix);
		shape_render(configName, M16);

		if (shapesEnabled)
		{
			updateShapes();

			int shapeID = -1;
			double depth = 0.0;

			if (shape_getTouched(configName, ref shapeID, ref depth))
			{
				if (touching == null || touching.GetInstanceID() != shapeID)
				{
					for (int ii = 0; ii < touchableObjects.Length; ii++)
					{
						if (shapeID == touchableObjects[ii].GetInstanceID())
						{
							touching = touchableObjects[ii];
							break;
						}
					}

				}
			}
			else
				touching = null;

			touchingDepth = (float)depth;
		}

		if (safetyMode)
		{
			safeUpdateManipulator();
			return;
		}


		// Pull the data from the device
		updateDevice();


		// Update the gamestate

		//Update the physics manipulator and whatnot.
		updateManipulator();

		//modified
		double[] relPos = new double[3];
		Vector3 worldPos;
		//Vector3 euler;
		//Vector3 worldEuler;
		Vector3 screenRel;
		Quaternion rhcQ;
		string message;
		float dist = distance / 2;





		Vector4 R1;
		Vector4 R2;
		Vector4 R3;
		Vector4 R4;

		int lrId;
		Matrix4x4 rotRaw = new Matrix4x4();
		Matrix4x4 Rot;


		if (configName == "Left Device")
		{
			relPos[0] = stylusPositionRaw.x - iniPosInput[0];
			relPos[1] = stylusPositionRaw.y - iniPosInput[1];
			relPos[2] = stylusPositionRaw.z - iniPosInput[2];

			screenRel.x = (float)relPos[0] - offset.x - dist;
			screenRel.y = (float)relPos[2] - offset.y;//relPos[2] is y
			screenRel.z = (float)relPos[1] - offset.z;//relPos[1] is z

			screenRel = screenOrientation.MultiplyVector(screenRel);
			relPos[0] = screenRel.x;
			relPos[1] = screenRel.y;
			relPos[2] = screenRel.z;

			worldPos.x = incision.x + (float)relPos[0] * motionScalingFactor;
			worldPos.y = incision.y + (float)relPos[1] * motionScalingFactor;
			worldPos.z = incision.z + (float)relPos[2] * motionScalingFactor;
			/*worldEuler.x = 90 + euler.x;
			worldEuler.y = -euler.y+180;
			worldEuler.z = euler.z;*/

			//Debug.Log(worldPos);
			//Debug.Log(worldEuler);


			lrId = 0;
		}
		else if (configName == "Right Device")
		{
			relPos[0] = stylusPositionRaw.x - iniPosInput[0];
			relPos[1] = stylusPositionRaw.y - iniPosInput[1];
			relPos[2] = stylusPositionRaw.z - iniPosInput[2];
			//print(relPos[1]);
			screenRel.x = (float)relPos[0] - offset.x + dist;
			screenRel.y = (float)relPos[2] - offset.y;
			screenRel.z = (float)relPos[1] - offset.z;

			screenRel = screenOrientation.MultiplyVector(screenRel);
			relPos[0] = screenRel.x;
			relPos[1] = screenRel.y;
			relPos[2] = screenRel.z;
			worldPos.x = incision.x + (float)relPos[0] * motionScalingFactor;
			worldPos.y = incision.y + (float)relPos[1] * motionScalingFactor;
			worldPos.z = incision.z + (float)relPos[2] * motionScalingFactor;
			lrId = 1;
		}
		else
		{
			/*worldPos.x = (float)relPos[0];
			worldPos.y = (float)relPos[1];
			worldPos.z = (float)relPos[2];
			lrId = 2;*/
			relPos[0] = stylusPositionRaw.x - iniPosInput[0];
			relPos[1] = stylusPositionRaw.y - iniPosInput[1];
			relPos[2] = stylusPositionRaw.z - iniPosInput[2];
			worldPos.x = incision.x - dist + (float)relPos[0] - offset.x;
			worldPos.y = incision.y + (float)relPos[2] - offset.y;
			worldPos.z = incision.z + (float)relPos[1] - offset.z;
			lrId = 0;

		}
		//Debug.Log(worldPos);

		string str1 = worldPos.x.ToString() + "," + worldPos.y.ToString() + "," + worldPos.z.ToString();

		//Debug.Log(stylusRotationWorld.ToString());
		rhcQ.w = stylusRotationWorld.w;
		rhcQ.x = stylusRotationWorld.x;
		rhcQ.y = stylusRotationWorld.z;
		rhcQ.z = stylusRotationWorld.y;

		float s = 1 / (rhcQ.w * rhcQ.w + rhcQ.x * rhcQ.x + rhcQ.y * rhcQ.y + rhcQ.z * rhcQ.z);
		//Debug.Log(rhcQ.ToString());
		//Debug.Log(s);
		R1.x = 1 - 2 * s * (rhcQ.y * rhcQ.y) - 2 * s * (rhcQ.z * rhcQ.z);
		R1.y = 2 * s * (rhcQ.x * rhcQ.y) - 2 * s * (rhcQ.w * rhcQ.z);
		R1.z = 2 * s * (rhcQ.x * rhcQ.z) + 2 * s * (rhcQ.w * rhcQ.y);
		R1.w = 0;

		R2.x = 2 * s * (rhcQ.x * rhcQ.y) + 2 * s * (rhcQ.w * rhcQ.z);
		R2.y = 1 - 2 * s * (rhcQ.x * rhcQ.x) - 2 * s * (rhcQ.z * rhcQ.z);
		R2.z = 2 * s * (rhcQ.y * rhcQ.z) - 2 * s * (rhcQ.w * rhcQ.x);
		R2.w = 0;

		R3.x = 2 * s * (rhcQ.x * rhcQ.z) - 2 * s * (rhcQ.w * rhcQ.y);
		R3.y = 2 * s * (rhcQ.y * rhcQ.z) + 2 * s * (rhcQ.w * rhcQ.x);
		R3.z = 1 - 2 * s * (rhcQ.x * rhcQ.x) - 2 * s * (rhcQ.y * rhcQ.y);
		R3.w = 0;

		R4.x = 0;
		R4.y = 0;
		R4.z = 0;
		R4.w = 1;

		rotRaw.SetRow(0, R1);
		rotRaw.SetRow(1, R2);
		rotRaw.SetRow(2, R3);
		rotRaw.SetRow(3, R4);
		Rot = iniOrientation * rotRaw * screenOrientation.transpose;
		//print(iniOrientation.ToString());
		//print(rotRaw.ToString());
		//print(rotRaw[9].ToString());
		//string str2 = euler.ToString();

		//changable variables
		double sf = 0.03;//force scaling factor
		double fd = 18000;//distance where force starts to work
		//variables used for calculation of force
		Vector3 x1;
		Vector3 x2;
		Vector3 x3;
		Vector3 perpend;
		Vector3 forcedir;
		Vector3 direction;
		Vector3 unit;

		double t;
		double dc;

		unit.x = 0;
		unit.y = 0;
		unit.z = 1;
		direction = Rot.MultiplyVector(unit);//world orientation 
											 //Debug.Log(direction.x);

		x1 = worldPos;

		//Debug.Log(Buttons[0].ToString());// Buttons[0] up button front, Buttons[1] up button back


		if (configName == "Left Device")
		{
			double[] zero = { 0.0, 0.0, 0.0 };
			double[] force = { 0.0, 0.0, 0.0 }; //right x up y front z

			x2.x = x1.x + direction.x;
			x2.y = x1.y + direction.y;
			x2.z = x1.z + direction.z;

			x3.x = rcm.x - x1.x;
			x3.y = rcm.y - x1.y;
			x3.z = rcm.z - x1.z;

			t = Vector3.Dot(x3, direction);

			perpend.x = x1.x + (float)t * direction.x;
			perpend.y = x1.y + (float)t * direction.y;
			perpend.z = x1.z + (float)t * direction.z;
			forcedir.x = rcm.x - perpend.x;
			forcedir.y = rcm.y - perpend.y;
			forcedir.z = rcm.z - perpend.z;
			forcedir = screenOrientation.transpose.MultiplyVector(forcedir);
			//Debug.Log(perpend.ToString());
			//Debug.Log(forcedir.ToString());



			dc = (worldPos.x - rcm.x) * (worldPos.x - rcm.x) + (worldPos.y - rcm.y) * (worldPos.y - rcm.y) + (worldPos.z - rcm.z) * (worldPos.z - rcm.z);
			//Debug.Log(dc.ToString());
			if (worldPos.z < rcm.z && dc < fd)
			{
				force[0] = sf * forcedir.x;
				force[1] = sf * forcedir.z;
				force[2] = sf * forcedir.y;
				if (force[0] < 3 && force[1] < 3 && force[2] < 3)
				{
					setForce(configName, force, zero);

				}
				else
				{
					setForce(configName, zero, zero);
				}
				//Debug.Log("low");
			}
			else
			{
				setForce(configName, zero, zero);
			}

		}
		else if (configName == "Right Device")
        {
			double[] zero = { 0.0, 0.0, 0.0 };
			double[] force = { 0.0, 0.0, 0.0 }; //right x up y front z

			x2.x = x1.x + direction.x;
			x2.y = x1.y + direction.y;
			x2.z = x1.z + direction.z;

			x3.x = rcm.x - x1.x;
			x3.y = rcm.y - x1.y;
			x3.z = rcm.z - x1.z;

			t = Vector3.Dot(x3, direction);

			perpend.x = x1.x + (float)t * direction.x;
			perpend.y = x1.y + (float)t * direction.y;
			perpend.z = x1.z + (float)t * direction.z;
			forcedir.x = rcm.x - perpend.x;
			forcedir.y = rcm.y - perpend.y;
			forcedir.z = rcm.z - perpend.z;
			forcedir = screenOrientation.transpose.MultiplyVector(forcedir);
			//Debug.Log(perpend.ToString());
			//Debug.Log(forcedir.ToString());



			dc = (worldPos.x - rcm.x) * (worldPos.x - rcm.x) + (worldPos.y - rcm.y) * (worldPos.y - rcm.y) + (worldPos.z - rcm.z) * (worldPos.z - rcm.z);
			//Debug.Log(dc.ToString());
			if (worldPos.z < rcm.z && dc < fd)
			{
				force[0] = sf * forcedir.x;
				force[1] = sf * forcedir.z;
				force[2] = sf * forcedir.y;
				if (force[0] < 3 && force[1] < 3 && force[2] < 3)
				{
					setForce(configName, force, zero);

				}
				else
				{
					setForce(configName, zero, zero);
				}
				//Debug.Log("low");
			}
			else
			{
				setForce(configName, zero, zero);
			}
		}
        else
        {
	
        }





		message = lrId.ToString() + "," + str1 + ",";
		for (int i = 0; i < 11; i++)
		{
			if (i % 4 != 3 && i != 10)
			{
				message = message + Rot[i] + ",";
			}
			else if (i == 10)
			{
				message = message + Rot[i];
			}



		}
		message += "," + Buttons[0].ToString() + "," + Buttons[1].ToString();// add button reading
		SendUDPData(message);
		Debug.Log(message);
		/*StringBuilder sbtest = new StringBuilder(256);
		string device_SerialNumber_Test;
		getDeviceSN("Default Device", sbtest, sbtest.Capacity);
		device_SerialNumber_Test = sbtest.ToString();
		print(device_SerialNumber_Test);*/


		checkErrors();
	}

	public void startSafetyMode()
	{
		safeUpdateManipulator();
		safetyMode = true;
	}
	public void endSafetyMode()
	{
		safetyMode = false;
	}
	public bool isInSafetyMode()
	{
		return safetyMode;
	}


	private static Quaternion QuaternionFromMatrix(Matrix4x4 m)
	{
		return Quaternion.LookRotation(m.GetColumn(2), m.GetColumn(1));
	}



	private void checkErrors()
	{
		StringBuilder sb = new StringBuilder(256);
		int code = getHDError(sb, sb.Capacity);
		if (code != 0)
		{
			Debug.LogError("Haptic(" + configName + ") HD_Error : " + code + ", " + sb.ToString());
			hapticErrorQueue.Enqueue(System.DateTime.Now.ToLongTimeString() + " - " + "Haptic(" + configName + ") HD_Error : " + code + ", " + sb.ToString());
		}

		code = getHLError(sb, sb.Capacity);
		if (code != 0)
		{
			Debug.LogError("Haptic(" + configName + ") HL_Error : " + code + ", " + sb.ToString());
			hapticErrorQueue.Enqueue(System.DateTime.Now.ToLongTimeString() + " - " + "Haptic(" + configName + ") HL_Error : " + code + ", " + sb.ToString());
		}

		while (hapticErrorQueue.Count > 100)
			hapticErrorQueue.Dequeue();
	}

	private void updateDevice()
	{
		if (isIncorrectVersion) return;

		// Retrieve the raw values
		double[] posInput = new double[3];
		double[] velInput = new double[3];
		getPosition(configName, posInput);
		getVelocity(configName, velInput);

		stylusPositionRaw.x = (float)posInput[0];
		stylusPositionRaw.y = (float)posInput[1];
		stylusPositionRaw.z = (float)posInput[2];

		stylusVelocityRaw.x = (float)velInput[0];
		stylusVelocityRaw.y = (float)velInput[1];
		stylusVelocityRaw.z = (float)velInput[2];

		double[] oriInput = new double[4];
		getProxyPosition(configName, posInput);
		proxyPositionRaw.x = (float)posInput[0];
		proxyPositionRaw.y = (float)posInput[1];
		proxyPositionRaw.z = (float)posInput[2];

		getProxyRotation(configName, oriInput);
		proxyOrientationRaw.x = (float)oriInput[0];
		proxyOrientationRaw.y = (float)oriInput[1];
		proxyOrientationRaw.z = (float)oriInput[2];
		proxyOrientationRaw.w = (float)oriInput[3];


		{
			double[] matInput = new double[16];
			getTransform(configName, matInput);
			Matrix4x4 mat;
			mat.m00 = (float)matInput[0];
			mat.m01 = (float)matInput[1];
			mat.m02 = (float)matInput[2];
			mat.m03 = (float)matInput[3];
			mat.m10 = (float)matInput[4];
			mat.m11 = (float)matInput[5];
			mat.m12 = (float)matInput[6];
			mat.m13 = (float)matInput[7];
			mat.m20 = (float)matInput[8];
			mat.m21 = (float)matInput[9];
			mat.m22 = (float)matInput[10];
			mat.m23 = (float)matInput[11];
			mat.m30 = (float)matInput[12];
			mat.m31 = (float)matInput[13];
			mat.m32 = (float)matInput[14];
			mat.m33 = (float)matInput[15];
			stylusTransformRaw = mat.transpose;
		}
		{
			double[] matInput = new double[16];
			getProxyTransform(configName, matInput);
			Matrix4x4 mat;
			mat.m00 = (float)matInput[0];
			mat.m01 = (float)matInput[1];
			mat.m02 = (float)matInput[2];
			mat.m03 = (float)matInput[3];
			mat.m10 = (float)matInput[4];
			mat.m11 = (float)matInput[5];
			mat.m12 = (float)matInput[6];
			mat.m13 = (float)matInput[7];
			mat.m20 = (float)matInput[8];
			mat.m21 = (float)matInput[9];
			mat.m22 = (float)matInput[10];
			mat.m23 = (float)matInput[11];
			mat.m30 = (float)matInput[12];
			mat.m31 = (float)matInput[13];
			mat.m32 = (float)matInput[14];
			mat.m33 = (float)matInput[15];
			proxyTransformRaw = mat.transpose;
		}



		getButtons(configName, Buttons, ref inkwell);


		// Cook the raw values.
		if (this.shapesEnabled)
		{
			stylusMatrixWorld = gameObject.transform.localToWorldMatrix * proxyTransformRaw;
			shape_enableShapeRendering();
		}
		else
		{
			stylusMatrixWorld = gameObject.transform.localToWorldMatrix * stylusTransformRaw;
			shape_disableShapeRendering();
		}
		stylusPositionWorld = stylusMatrixWorld.GetColumn(3);
		stylusRotationWorld = QuaternionFromMatrix(stylusMatrixWorld);

		stylusVelocityWorld = gameObject.transform.InverseTransformVector(stylusVelocityRaw);

	}


	private GameObject previousManipulator = null;

	private void updateManipulator()
	{
		if (this.hapticManipulator == null)
		{
			double[] zero = { 0.0, 0.0, 0.0 };
			setSpringStiffness(configName, 0.0, 0.0);
			return;
		}

		if (PhysicsManipulationEnabled == false || hapticManipulator != previousManipulator)
		{ // No physics, just move it.
			hapticManipulator.transform.rotation = stylusRotationWorld;
			hapticManipulator.transform.position = stylusPositionWorld;
			setSpringStiffness(configName, 0.0, 0.0);
			previousManipulator = hapticManipulator;
			return;
		}
		previousManipulator = hapticManipulator;

		//ConfigurableJoint joint = hapticManipulator.GetComponent<ConfigurableJoint>();
		Rigidbody body = hapticManipulator.GetComponent<Rigidbody>();

		// FIXME! Why 10? Determine this from bounding sphere.
		if ((body.position - stylusPositionWorld).magnitude > 10)
		{
			body.position = stylusPositionWorld;
			body.velocity = stylusVelocityWorld;
			body.rotation = stylusRotationWorld;
		}
		Vector3 springPos = gameObject.transform.InverseTransformPoint(this.hapticManipulator.transform.position);
		double[] springPosOut = new double[3];
		springPosOut[0] = springPos.x;
		springPosOut[1] = springPos.y;
		springPosOut[2] = springPos.z;

		Vector3 springVel = gameObject.transform.InverseTransformVector(body.velocity);
		double[] springVelOut = new double[3];
		springVelOut[0] = springVel.x;
		springVelOut[1] = springVel.y;
		springVelOut[2] = springVel.z;
		setSpringAnchorPosition(configName, springPosOut, springVelOut);
		setSpringStiffness(configName, this.PhysicsForceStrength, 0);


		{
			// Idea : (Could we take velocity from the device?)

			Vector3 force = new Vector3();
			force = (stylusPositionWorld) - body.position;
			force *= body.mass;
			force *= 1000;
			force *= (0.15f); // Magic number So that manupilator with mass 1.0 feels right.

			Vector3 V = new Vector3();
			V = (body.velocity);
			V *= V.magnitude;

			force -= (body.mass * V * this.PhysicsForceDamping);


			body.AddForce(force);
		}

		//Rotation

		{
			// Apply rotation to this item.
			Vector3 torque = DetermineTorque(body);

			body.AddTorque(torque, ForceMode.VelocityChange);

			// Apply rotation to anything we're grabbing.
			FixedJoint[] joints = hapticManipulator.GetComponentsInChildren<FixedJoint>();
			foreach (FixedJoint J in joints)
			{
				if (J.connectedBody != null)
				{
					J.connectedBody.AddTorque(torque, ForceMode.VelocityChange);
				}
			}
		}
	}

	private Vector3 DetermineTorque(Rigidbody body)
	{
		if (body == null)
			return Vector3.zero;

		Quaternion AngleDifference = stylusRotationWorld * Quaternion.Inverse(body.rotation);

		float AngleToCorrect = Quaternion.Angle(body.rotation, stylusRotationWorld);
		Vector3 Perpendicular = Vector3.Cross(transform.up, transform.forward);
		if (Vector3.Dot(stylusRotationWorld * Vector3.forward, Perpendicular) < 0)
			AngleToCorrect *= -1;
		Quaternion Correction = Quaternion.AngleAxis(AngleToCorrect, transform.up);

		Vector3 MainRotation = RectifyAngleDifference((AngleDifference).eulerAngles);
		Vector3 CorrectiveRotation = RectifyAngleDifference((Correction).eulerAngles);

		Vector3 torque = ((MainRotation - CorrectiveRotation / 2) - body.angularVelocity);
		return torque;
	}
	private Vector3 RectifyAngleDifference(Vector3 angdiff)
	{
		if (angdiff.x > 180) angdiff.x -= 360;
		if (angdiff.y > 180) angdiff.y -= 360;
		if (angdiff.z > 180) angdiff.z -= 360;
		return angdiff;
	}


	// Put the cursor on the device so it doesn't jump.
	public void safeUpdateManipulator()
	{
		if (isIncorrectVersion) return;

		//Debug.unityLogger.Log("Safing the Manipulator.");

		if (hapticManipulator == null)
			return;
		Rigidbody body = hapticManipulator.GetComponent<Rigidbody>();
		if (body == null)
			return;
		updateDevice();

		{
			body.position = stylusPositionWorld;
			body.rotation = stylusRotationWorld;
			body.velocity = Vector3.zero;
		}

		double[] zero = { 0.0, 0.0, 0.0 };
		setSpringStiffness(configName, 0.0, 0.0);
		setForce(configName, zero, zero);
	}



	void setupShapes()
	{
		if (isIncorrectVersion) return;

		touchableObjects = GameObject.FindGameObjectsWithTag("Touchable") as GameObject[];

		for (int ii = 0; ii < touchableObjects.Length; ii++)
		{
			int shapeID = touchableObjects[ii].GetInstanceID();
			string name = touchableObjects[ii].name;

			GameObject go = touchableObjects[ii];
			Mesh mesh = null;

			// If the object has a collision mesh, use that.
			MeshCollider collider = go.GetComponent<MeshCollider>();
			if (collider != null)
			{
				mesh = collider.sharedMesh;
			}
			if (mesh == null)
			{
				MeshFilter filter = go.GetComponent<MeshFilter>();
				if (filter != null)
					mesh = filter.mesh;
			}

			// Vectors need to be converted to array of primatives. 
			// Triangles already are an array of ints.
			if (mesh != null)
			{
				double[] vertices = Vector3ArrayToDoubleArray(mesh.vertices);
				int[] triangles = mesh.triangles;

				shape_define(shapeID, name, vertices, triangles, vertices.Length, triangles.Length);
			}

		} //for(ii)

		// TODO Update the transforms of all shapes.

	}

	void updateShapes()
	{
		if (isIncorrectVersion) return;

		GameObject[] myObjects = GameObject.FindGameObjectsWithTag("Touchable") as GameObject[];

		for (int ii = 0; ii < myObjects.Length; ii++)
		{
			int shapeID = myObjects[ii].GetInstanceID();

			Matrix4x4 M = myObjects[ii].transform.localToWorldMatrix;

			M = this.transform.worldToLocalMatrix * M;

			double[] M16 = MatrixToDoubleArray(M);

			shape_setTransform(shapeID, M16);
		}
	}

	//Convert Vector3[] to double[]  
	//Used by setupShapes to cook the vertex array of a mesh.
	private static double[] Vector3ArrayToDoubleArray(Vector3[] inVectors)
	{
		double[] outDoubles = new double[inVectors.Length * 3];

		for (int ii = 0; ii < inVectors.Length; ii++)
		{
			outDoubles[3 * ii + 0] = inVectors[ii].x;
			outDoubles[3 * ii + 1] = inVectors[ii].y;
			outDoubles[3 * ii + 2] = inVectors[ii].z;
		}

		return outDoubles;
	}
	//Convert Matrix4x4 to double[]  
	private static double[] MatrixToDoubleArray(Matrix4x4 M)
	{
		double[] out16 = new double[16];

		out16[0] = M.m00;
		out16[1] = M.m10;
		out16[2] = M.m20;
		out16[3] = M.m30;

		out16[4] = M.m01;
		out16[5] = M.m11;
		out16[6] = M.m21;
		out16[7] = M.m31;

		out16[8] = M.m02;
		out16[9] = M.m12;
		out16[10] = M.m22;
		out16[11] = M.m32;

		out16[12] = M.m03;
		out16[13] = M.m13;
		out16[14] = M.m23;
		out16[15] = M.m33;
		/*
		out16 [0] = M.m00;
		out16 [1] = M.m01;
		out16 [2] = M.m02;
		out16 [3] = M.m03;

		out16 [4] = M.m10;
		out16 [5] = M.m11;
		out16 [6] = M.m12;
		out16 [7] = M.m13;

		out16 [8] = M.m20;
		out16 [9] = M.m21;
		out16 [10] =M.m22;
		out16 [11] =M.m23;

		out16 [12] =M.m30;
		out16 [13] =M.m31;
		out16 [14] =M.m32;
		out16 [15] =M.m33;*/

		return out16;
	}


	public string[] retrieveErrorList()
	{
		string[] output = new string[hapticErrorQueue.Count];
		hapticErrorQueue.CopyTo(output, 0);
		return output;
	}
	public void clearErrorQueue()
	{
		hapticErrorQueue.Clear();
	}


	private Rect windowRect = new Rect(20, 20, 350, 125);
	void OnGUI()
	{
		if (showOldVersionPopup)
			windowRect = GUI.Window(0, windowRect, OldVersionDialogWindow, "Out-of-date OpenHaptics version.");
		else if (showNoDevicePopup)
			windowRect = GUI.Window(0, windowRect, NoDeviceDialogWindow, "Haptic Device Not Found.");

	}

	// This is the actual window.
	void NoDeviceDialogWindow(int windowID)
	{
		float y = 30;

		GUI.Label(new Rect(5, y, windowRect.width, 20), "Could not find haptic device named \"" + configName + "\""); y += 30;

		if (GUI.Button(new Rect(windowRect.width * 0.33f, y, windowRect.width * 0.33f, 20), "OK"))
		{
			showNoDevicePopup = false;
		}
		y += 30;

		if (GUI.Button(new Rect(windowRect.width * 0.33f, y, windowRect.width * 0.33f, 20), "Exit"))
		{
			Application.Quit();
			showNoDevicePopup = false;
#if UNITY_EDITOR
			UnityEditor.EditorApplication.isPlaying = false;
#endif
		}
		y += 30;
	}
	void OldVersionDialogWindow(int windowID)
	{
		// Get Version String
		StringBuilder sb = new StringBuilder(256);
		getVersionString(sb, sb.Capacity);

		string ver = sb.ToString().Substring(sb.ToString().IndexOf("OpenHapticsVersion=") + "OpenHapticsVersion=".Length);


		float y = 30;

		GUI.Label(new Rect(5, y, windowRect.width, 20), "Unity Plugin requires OpenHaptics version 3.50.0"); y += 30;
		GUI.Label(new Rect(5, y, windowRect.width, 20), "(Current OH Version : " + ver + ")"); y += 30;

		if (GUI.Button(new Rect(windowRect.width * 0.33f, y, windowRect.width * 0.33f, 20), "Exit"))
		{
			Application.Quit();
			showNoDevicePopup = false;
#if UNITY_EDITOR
			UnityEditor.EditorApplication.isPlaying = false;
#endif
		}
		y += 30;

	}

	//code from: http://kimdicks.blogspot.com/2017/11/unityudp.html
	void SendUDPData(string tempData)
	{
		sendByte = System.Text.Encoding.UTF8.GetBytes(tempData);
		udpClient.Send(sendByte, sendByte.Length, ipEndPoint);
	}

	// code from: https://stackoverflow.com/questions/12088610/conversion-between-euler-quaternion-like-in-unity3d-engine
	public static Vector3 FromQ2(Quaternion q1)
	{
		float sqw = q1.w * q1.w;
		float sqx = q1.x * q1.x;
		float sqy = q1.y * q1.y;
		float sqz = q1.z * q1.z;
		float unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
		float test = q1.x * q1.w - q1.y * q1.z;
		Vector3 v;

		if (test > 0.4995f * unit)
		{ // singularity at north pole
			v.y = 2f * Mathf.Atan2(q1.y, q1.x);
			v.x = Mathf.PI / 2;
			v.z = 0;
			return NormalizeAngles(v * Mathf.Rad2Deg);
		}
		if (test < -0.4995f * unit)
		{ // singularity at south pole
			v.y = -2f * Mathf.Atan2(q1.y, q1.x);
			v.x = -Mathf.PI / 2;
			v.z = 0;
			return NormalizeAngles(v * Mathf.Rad2Deg);
		}
		Quaternion q = new Quaternion(q1.w, q1.z, q1.x, q1.y);
		v.y = (float)Mathf.Atan2(2f * q.x * q.w + 2f * q.y * q.z, 1 - 2f * (q.z * q.z + q.w * q.w));     // Yaw
		v.x = (float)Mathf.Asin(2f * (q.x * q.z - q.w * q.y));                             // Pitch
		v.z = (float)Mathf.Atan2(2f * q.x * q.y + 2f * q.z * q.w, 1 - 2f * (q.y * q.y + q.z * q.z));      // Roll
		return NormalizeAngles(v * Mathf.Rad2Deg);
	}
	static Vector3 NormalizeAngles(Vector3 angles)
	{
		angles.x = NormalizeAngle(angles.x);
		angles.y = NormalizeAngle(angles.y);
		angles.z = NormalizeAngle(angles.z);
		return angles;
	}

	static float NormalizeAngle(float angle)
	{
		while (angle > 360)
			angle -= 360;
		while (angle < 0)
			angle += 360;
		return angle;
	}



#if UNITY_EDITOR
	// In editor Gizmos
	void OnDrawGizmos()
	{
		if (hHD >= 0)
		{
			// Draw Extants
			{
				const int minX = 0;
				const int minY = 1;
				const int minZ = 2;
				const int maxX = 3;
				const int maxY = 4;
				const int maxZ = 5;

				Vector3 usableBox = new Vector3 (
					                    (float)(usable_extents [maxX] - usable_extents [minX]),
					                    (float)(usable_extents [maxY] - usable_extents [minY]),
					                    (float)(usable_extents [maxZ] - usable_extents [minZ]));
				Vector3 usableCenter = new Vector3 (
					                       0.5f * (float)(usable_extents [maxX] + usable_extents [minX]),
					                       0.5f * (float)(usable_extents [maxY] + usable_extents [minY]),
					                       0.5f * (float)(usable_extents [maxZ] + usable_extents [minZ]));
			
				Gizmos.color = Color.green;
				Gizmos.matrix = gameObject.transform.localToWorldMatrix;
				Gizmos.DrawWireCube(usableCenter, usableBox); 

				Vector3 maxBox = new Vector3 (
					                 (float)(max_extents [maxX] - max_extents [minX]),
					                 (float)(max_extents [maxY] - max_extents [minY]),
					                 (float)(max_extents [maxZ] - max_extents [minZ]));
				Vector3 maxCenter = new Vector3 (
					                    0.5f * (float)(max_extents [maxX] + max_extents [minX]),
					                    0.5f * (float)(max_extents [maxY] + max_extents [minY]),
					                    0.5f * (float)(max_extents [maxZ] + max_extents [minZ]));


				Gizmos.color = Color.yellow;
				Gizmos.matrix = gameObject.transform.localToWorldMatrix;
				Gizmos.DrawWireCube(maxCenter, maxBox); 
			}

			// Draw Stylus!
			Gizmos.color = Color.white;
			Gizmos.matrix = gameObject.transform.localToWorldMatrix * stylusTransformRaw;
			Gizmos.DrawWireSphere(Vector3.zero, 10);
			Gizmos.DrawWireCube(new Vector3 (0, 0, 20), new Vector3 (5, 5, 40));

			// Draw Buttons

			if (Buttons [0] == 1)
				Gizmos.color = Color.green;
			else
				Gizmos.color = Color.gray;
			Gizmos.DrawWireSphere(new Vector3 (0, 0, 5), 2f);
			if (Buttons [1] >= 0)
			{
				if (Buttons [1] == 1)
					Gizmos.color = Color.green;
				else
					Gizmos.color = Color.gray;
				Gizmos.DrawWireSphere(new Vector3 (0, 0, 10), 2f);
			}
		} else
		{
			// Else no connection.
			Vector3 OmniBox = new Vector3(160,120,70);
			Gizmos.color = Color.grey;
			Gizmos.matrix = gameObject.transform.localToWorldMatrix;
			Gizmos.DrawWireCube (Vector3.zero, OmniBox);


		}
	} //OnDrawGizmos()
#endif

} //class HapticPlugin





/*public class DisplayOnlyAttribute : PropertyAttribute
{

}*/


/*#if UNITY_EDITOR
[CustomPropertyDrawer(typeof(DisplayOnlyAttribute))]
public class ReadOnlyDrawer : PropertyDrawer
{
	public override float GetPropertyHeight(SerializedProperty property,
		GUIContent label)
	{
		return EditorGUI.GetPropertyHeight(property, label, true);
	}

	public override void OnGUI(Rect position,
		SerializedProperty property,
		GUIContent label)
	{
		GUI.enabled = false;
		EditorGUI.PropertyField(position, property, label, true);
		GUI.enabled = true;
	}
}
#endif*/


using System.Collections;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine;
using Random = UnityEngine.Random;
using System;



public class CarAgent2 : Agent
{

    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    //public List<AxleInfo> axleInfos; // the information about each individual axle
    public float maxMotorTorque; // maximum torque the motor can apply to wheel
    public float maxSteeringAngle; // maximum steer angle the wheel can have
    public float maxBrakeTorque;
    //public bool motor; // is this wheel attached to motor?
    //public bool steering; // does this wheel apply steer angle?
    private Rigidbody _rigidBody;
    public float velocity; //for debugging
    public float reward;
    public string lane;
    public float leftDistance;
    public float rightDistance;
    void Start()
    {
        _rigidBody = GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin()
    {
        leftWheel.motorTorque = 0f;
        rightWheel.motorTorque = 0f;
        leftWheel.steerAngle = 0f;
        rightWheel.steerAngle = 0f;
        int spawnIndex = Random.Range(0,3);
        if (lane=="inner")
        {
            if (spawnIndex == 0)
            {
                this.transform.localPosition = new Vector3 (75.55f,0.5f,0f);
                this.transform.eulerAngles = new Vector3(0f, 373.76f, 0f);
            }
            else if (spawnIndex == 1)
            {
                this.transform.localPosition = new Vector3 (50.8f,0.5f,22.5f);
                this.transform.eulerAngles = new Vector3(0f, 186f, 0f);
            }
            else if (spawnIndex == 2)
            {
                this.transform.localPosition = new Vector3 (1.674f,0.5f,-100f);
                this.transform.eulerAngles = new Vector3(0f, 83.824f, 0f);
            }

        }
        else if (lane=="outer")
        {
            if (spawnIndex == 0)
            {
                this.transform.localPosition = new Vector3 (78.6f,0.5f,0f);
                this.transform.eulerAngles = new Vector3(0f, -164f, 0f);
            }
            else if (spawnIndex == 1)
            {
                this.transform.localPosition = new Vector3 (48.1f,0.5f,25.5f);
                this.transform.eulerAngles = new Vector3(0f, 0f, 0f);
            }
            else if (spawnIndex == 2)
            {
                this.transform.localPosition = new Vector3 (11f,0.5f,-101f);
                this.transform.eulerAngles = new Vector3(0f,-112.25f, 0f);
            }
        }
        _rigidBody.velocity = Vector3.zero; 
        _rigidBody.angularVelocity = Vector3.zero;
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        leftWheel.steerAngle = actionBuffers.ContinuousActions[0] * maxSteeringAngle;
        rightWheel.steerAngle = actionBuffers.ContinuousActions[0]* maxSteeringAngle;

        leftWheel.motorTorque = Mathf.Clamp(actionBuffers.ContinuousActions[1],0.99f,1f)  * maxMotorTorque;
        rightWheel.motorTorque = Mathf.Clamp(actionBuffers.ContinuousActions[1],0.99f,1f) * maxMotorTorque;
        velocity = this.transform.InverseTransformDirection(this._rigidBody.velocity).z; //for debugging
        //leftWheel.brakeTorque = actionBuffers.ContinuousActions[2] * maxBrakeTorque;
        //rightWheel.brakeTorque = actionBuffers.ContinuousActions[2] * maxBrakeTorque;
        //Debug.Log("Motor Torque:" + actionBuffers.ContinuousActions[1] + ",  Steering Angle:" + actionBuffers.ContinuousActions[0] + ", Brake Torque:" + actionBuffers.ContinuousActions[2]);
        //Debug.Log("Velocity:" + _rigidBody.velocity + "->" + _rigidBody.velocity.magnitude);
        
    }
    
    public override void Heuristic(in ActionBuffers actionsOut)
    {   
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal");
        continuousActionsOut[1] = Mathf.Clamp(Input.GetAxis("Vertical"),-1f,1f);
        //continuousActionsOut[2] = Input.GetAxis("Jump");
        //Debug.Log("Motor Torque:" + continuousActionsOut[1] + ",  Steering Angle:" + continuousActionsOut[0] + ", Brake Torque:" + continuousActionsOut[2]);

    }


    private void OnTriggerStay(Collider other)
    {
        if (lane == "inner")
        {
            if (other.gameObject.tag == "InnerAvoid2" || other.gameObject.tag == "CommonAvoid2")
            {
                reward = (this.transform.InverseTransformDirection(this._rigidBody.velocity).z)*0.03f;
                AddReward(reward);
            }
        }
        else if (lane == "outer")
        {
            if (other.gameObject.tag == "OuterAvoid2" || other.gameObject.tag == "CommonAvoid2")
            {
                reward = (this.transform.InverseTransformDirection(this._rigidBody.velocity).z)*0.03f;
                AddReward(reward);
            }
        }
    }

    private void OnCollisionEnter(Collision other)
    {
        if (other.gameObject.tag == "InnerEdge32" || other.gameObject.tag == "OuterEdge32")
        {
            AddReward(-10f);
            EndEpisode();
        }
        if (other.gameObject.tag == "Car")
        {
            AddReward(-10f);
            EndEpisode();
        }
        if (other.gameObject.tag == "Obstacle")
        {
            AddReward(-10f);
            EndEpisode();
        }
    }
    public override void CollectObservations(VectorSensor sensor)
    {

    }

    public void FixedUpdate()
    {        
        RaycastHit hit;
        RaycastHit righthit;
        Vector3 left =  -this.transform.right;
        Vector3 right = this.transform.right;



        Debug.DrawRay(transform.position, left*20f, Color.red);
        Debug.DrawRay(transform.position, right*20f, Color.red);
        if (Physics.Raycast(this.transform.position,right,out righthit, 20))
        {
            if (lane=="inner")
            {
                if (righthit.transform.tag == "OuterEdge32")
                {
                    rightDistance = righthit.distance;
                }
                else if (righthit.transform.tag == "InnerEdge32")
                {
                    rightDistance = 0f;

                }
            }
            else if (lane == "outer")
            {
                if (righthit.transform.tag == "InnerEdge32")
                {
                    rightDistance = righthit.distance;
                }
                else if (righthit.transform.tag == "OuterEdge32")
                {
                    rightDistance = 0f;

                }
            }
        }
        if (Physics.Raycast(this.transform.position,left,out hit, 20))
        {   
            if (lane=="inner")
            {
                if (hit.transform.tag == "InnerEdge32")
                {
                    leftDistance = hit.distance;
                }
                else if (hit.transform.tag == "OuterEdge32")
                {
                    leftDistance = 0f;
                    AddReward(-0.05f);
                }
            }
            else if (lane == "outer")
            {
                if (hit.transform.tag == "OuterEdge32")
                {
                    leftDistance = hit.distance;
                }
                else if (hit.transform.tag == "InnerEdge32")
                {
                    leftDistance = 0f;
                    AddReward(-0.05f);
                }
            }
        }
        // Right ray: hit
        // Left ray: no hit
        if (!Physics.Raycast(this.transform.position,left,out hit, 20) && Physics.Raycast(this.transform.position,right,out righthit, 20))
        {
            leftDistance = 0f;
            
            if (rightDistance<7.8f && rightDistance>7.2f && Vector3.Dot(-(righthit.point - this.transform.localPosition).normalized,righthit.normal)>0.98f)
            {
                reward = (this.transform.InverseTransformDirection(this._rigidBody.velocity).z)*0.01839f;
                AddReward(reward);
            }
            else
            {
                reward = -(this.transform.InverseTransformDirection(this._rigidBody.velocity).z)*0.01839f;
                AddReward(reward);
            }
        }
        // Right ray: no hit
        // Left ray: hit
        if (!Physics.Raycast(this.transform.position,right,out righthit, 20) && Physics.Raycast(this.transform.position,left,out hit, 20))
        {
            rightDistance = 0f;
            if (Vector3.Dot(-(hit.point - this.transform.localPosition).normalized,hit.normal)>0.98f)
            {
                if (leftDistance<=4.3f && leftDistance>=3.5f)
                {
                    reward = (this.transform.InverseTransformDirection(this._rigidBody.velocity).z)*(Mathf.Exp(-Mathf.Pow((Mathf.Abs(leftDistance - 4.5f)) / 0.2f, 2)) / 20f);
                    AddReward(reward);
                }
                else if (leftDistance<=5.4f && leftDistance>=4.7f)
                {
                    reward = (this.transform.InverseTransformDirection(this._rigidBody.velocity).z)*(Mathf.Exp(-Mathf.Pow((Mathf.Abs(leftDistance - 4.5f)) / 0.2f, 2)) / 20f);
                    AddReward(reward);
                }
                else if (leftDistance<3.5f)
                {
                    reward = (0.02f / 3.5f) * (leftDistance - 3.5f);
                    AddReward(reward);
                }
                else if (leftDistance>5.4f)
                {
                    reward = -(0.02f / 3.5f) * (leftDistance-5.4f);
                    AddReward(reward);
                }
                else if (leftDistance<4.7f && leftDistance>4.3f)
                {
                    reward = (this.transform.InverseTransformDirection(this._rigidBody.velocity).z)*0.01839f;
                    AddReward(reward);
                }
                /*else if (leftDistance<4.7f && leftDistance>4.3f)
                {
                    reward = (this.transform.InverseTransformDirection(this._rigidBody.velocity).z)*(Mathf.Exp(-Mathf.Pow((Mathf.Abs(leftDistance - 4.5f)) / 0.2f, 2)) / 20f);
                    AddReward(reward);
                }*/
            }
            
        }
        // Right ray: hit
        // Left ray: hit
        if (Physics.Raycast(this.transform.position,right,out righthit, 20) && Physics.Raycast(this.transform.position,left,out hit, 20))
        {
            if (rightDistance<7.8f && rightDistance>7.2f)
            {
                if (leftDistance<=4.3f && leftDistance>=3.5f)
                {
                    reward = (this.transform.InverseTransformDirection(this._rigidBody.velocity).z)*(Mathf.Exp(-Mathf.Pow((Mathf.Abs(leftDistance - 4.5f)) / 0.2f, 2)) / 20f);
                    AddReward(reward);
                }
                else if (leftDistance<=5.4f && leftDistance>=4.7f)
                {
                    reward = (this.transform.InverseTransformDirection(this._rigidBody.velocity).z)*(Mathf.Exp(-Mathf.Pow((Mathf.Abs(leftDistance - 4.5f)) / 0.2f, 2)) / 20f);
                    AddReward(reward);
                }
                else if (leftDistance<3.5f)
                {
                    reward = (0.02f / 3.5f) * (leftDistance - 3.5f);
                    AddReward(reward);
                }
                else if (leftDistance>5.4f)
                {
                    reward = -(0.02f / 3.5f) * (leftDistance-5.4f);
                    AddReward(reward);
                }
                else if (leftDistance<4.7f && leftDistance>4.3f)
                {
                    reward = (this.transform.InverseTransformDirection(this._rigidBody.velocity).z)*0.01839f;
                    AddReward(reward);
                }
                /*else if (leftDistance<4.7f && leftDistance>4.3f)
                {
                    reward = (this.transform.InverseTransformDirection(this._rigidBody.velocity).z)*(Mathf.Exp(-Mathf.Pow((Mathf.Abs(leftDistance - 4.5f)) / 0.2f, 2)) / 20f);
                    AddReward(reward);
                }*/
            }
            else
            {
                if (leftDistance<=4.3f && leftDistance>=3.5f)
                {
                    reward = (this.transform.InverseTransformDirection(this._rigidBody.velocity).z)*(Mathf.Exp(-Mathf.Pow((Mathf.Abs(leftDistance - 4.5f)) / 0.2f, 2)) / 20f);
                    AddReward(reward);
                }
                else if (leftDistance<=5.4f && leftDistance>=4.7f)
                {
                    reward = (this.transform.InverseTransformDirection(this._rigidBody.velocity).z)*(Mathf.Exp(-Mathf.Pow((Mathf.Abs(leftDistance - 4.5f)) / 0.2f, 2)) / 20f);
                    AddReward(reward);
                }
                else if (leftDistance<3.5f)
                {
                    reward = (0.02f / 3.5f) * (leftDistance - 3.5f);
                    AddReward(reward);
                }
                else if (leftDistance>5.4f)
                {
                    reward = -(0.02f / 3.5f) * (leftDistance-5.4f);
                    AddReward(reward);
                }
                else
                {
                    reward = 0f;
                }
            }
        }

        
        ApplyLocalPositionToVisuals(leftWheel);
        ApplyLocalPositionToVisuals(rightWheel);
    }
    public void ApplyLocalPositionToVisuals(WheelCollider collider)
    {
        if (collider.transform.childCount == 0)
        {
            return;
        }

        Transform visualWheel = collider.transform.GetChild(0);

        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);

        visualWheel.transform.position = position;
        visualWheel.transform.rotation = rotation;
    }
}

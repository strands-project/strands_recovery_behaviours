#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> Server;
ros::Subscriber robot_pose;

Server *server;
ros::Subscriber scan_sub;
float desiredAngle = -1000;
bool enableBackoff = false;
ros::NodeHandle *n;

typedef enum{
	IDLE,
	BACKOFF,
	TURN,
	FINAL,
	STOPPING,
	PREEMPTED,
	SUCCESS,
	FAIL
}EClimbState;

const char *stateStr[] = {"Idle","Backoff","turn","progress","final","stopping","preempted","success","fail"};

EClimbState state = IDLE;
EClimbState lastState = IDLE;
ros::Publisher cmd_vel;
geometry_msgs::Twist base_cmd;

float normalizeAngle(float angle)
{
	while (angle < -M_PI) angle += 2*M_PI;
	while (angle > +M_PI) angle -= 2*M_PI;
	return angle;
}

float saturate(float num,float limit)
{
	if (num > +limit) return +limit;
	if (num < -limit) return -limit;
	return num;
}

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	/*turning detector*/
	if (state == TURN)
	{
		float currentAngle = normalizeAngle(tf::getYaw(msg->orientation));
		if (desiredAngle < -10) desiredAngle = normalizeAngle(currentAngle+M_PI);
		base_cmd.linear.x = 0;  
		base_cmd.angular.z = saturate((desiredAngle-currentAngle)*0.5,0.3);
		if (fabs(desiredAngle-currentAngle) < 0.1){
			base_cmd.angular.z = 0;
			state = SUCCESS;
		} 
		cmd_vel.publish(base_cmd);
	}
}

     
void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
	if (state == BACKOFF)
	{
		int filt = 10;
		int num_ranges = (int)scan_msg->ranges.size();
		float x[num_ranges];
		float y[num_ranges];
		float d[num_ranges];
		float m[num_ranges];
		float angle;
		for (int i = 0; i <= num_ranges; i++){
			angle = scan_msg->angle_min+i*scan_msg->angle_increment;
			x[i] = scan_msg->ranges[i]*cos(angle)+0.10;
			y[i] = scan_msg->ranges[i]*sin(angle);
			d[i] = sqrt(x[i]*x[i]+y[i]*y[i]);
			m[i] = true;
		}
		m[0] = d[0];
		for (int i = 1; i < filt; i++) m[i]=(m[i-1]+d[i]);
		float minDist = 100000.0;
		int mindex = 0;
		for (int i = filt; i < num_ranges; i++)m[i]=(m[i-1]+d[i]-d[i-filt]);

		for (int i = filt; i < num_ranges; i++)
		{
			 if (m[i] < minDist)
			{
				minDist = m[i];
				mindex = i;
			}
		}

		float currentAngle = scan_msg->angle_min+mindex*scan_msg->angle_increment;
		//printf("%i %.3f %.3f\n",mindex,currentAngle,minDist/filt);
		if (fabs(currentAngle) < 0.2) base_cmd.linear.x = -0.05; else base_cmd.linear.x = 0.00;
		base_cmd.angular.z = currentAngle*0.5;
		if (minDist/filt > 0.55){
			base_cmd.linear.x = 0.0;
			state = TURN;
			desiredAngle = -100;
		}
		cmd_vel.publish(base_cmd);
	}
}

void actionServerCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal, Server* as)
{
	move_base_msgs::MoveBaseResult result;
	n->param("/recovery/enable_backoff",enableBackoff,false);
	if (enableBackoff)
	{
		state = BACKOFF;
		float goalX = goal->target_pose.pose.position.x;
		float goalY = goal->target_pose.pose.position.y;
		//if (goalX == 0 && goalY == 0) 
		while (state == BACKOFF ||state == TURN || state == FINAL)
		{
			if (state == FINAL) state = SUCCESS;
			if (state == PREEMPTED) state =  FAIL;
			usleep(15000);
		}
	}
	if (state == IDLE) server->setAborted(result);
	if (state == SUCCESS) server->setSucceeded(result);
	if (state == FAIL) server->setAborted(result);
	if (state == PREEMPTED) server->setPreempted(result);
	state = STOPPING;

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "backoff_behaviour");
	n = new ros::NodeHandle;
	state = IDLE;
	lastState = IDLE;
	cmd_vel = n->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	server = new Server(*n, "backoff_behaviour", boost::bind(&actionServerCallback, _1, server), false);
	server->start();
	scan_sub = n->subscribe("scan", 100, scanCallback);
	robot_pose = n->subscribe("/robot_pose", 1000, poseCallback);
	while (ros::ok()){
		if (server->isPreemptRequested() && state != IDLE) state = PREEMPTED;
		if (state != lastState) printf("%s\n",stateStr[state]);
		lastState = state;
		if (state == STOPPING){
			base_cmd.linear.x = base_cmd.angular.z = 0;
			cmd_vel.publish(base_cmd);
			state = IDLE;
		} 
		ros::spinOnce();
		usleep(30000);
	}
}

//
// Created by yezi on 2021/3/6.
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <cstdio>
#include <unistd.h>
#include <termios.h>

#include <map>

// Map for movement keys
std::map<char, std::vector<float>> moveBindings
    {
        {'q', {1, 1, 0}},
        {'w', {1.9, 0, 0}},
        {'e', {1, -1, 0}},
        {'a', {0, 1.9, 0}},
        {'s', {0, 0, 0}},
        {'d', {0, -1.9, 0}},
        {'z', {-1, 1, 0}},
        {'x', {-1.9, 0, 0}},
        {'c', {-1, -1, 0}},
        {'Q', {1.9, 1.9, 0}},
        {'W', {3.81, 0, 0}},
        {'E', {1.9, -1.9, 0}},
        {'A', {0, 3.81, 0}},
        {'S', {0, 0, 0}},
        {'D', {0, -3.81, 0}},
        {'Z', {-1.9, 1.9, 0}},
        {'X', {-3.81, 0, 0}},
        {'C', {-1.9, -1.9, 0}},
        {'f', {0, 0, 3.5}},
        {'g', {0, 0, -3.5}},
        {'F', {0, 0, 7}},
        {'G', {0, 0, -7}},
    };

// Init variables
float x(0), y(0), th(0); // Forward/backward/neutral direction vars
char key(' ');

// For non-blocking keyboard inputs
int getch(void) {
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int main(int argc, char **argv) {
  // Init ROS node
  ros::init(argc, argv, "teleop_twist_keyboard");
  ros::NodeHandle nh;

  // Init cmd_vel publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // Create Twist message
  geometry_msgs::Twist twist;

  while (ros::ok()) {

    // Get the pressed key
    key = getch();

    // If the key corresponds to a key in moveBindings
    if (moveBindings.count(key) == 1) {
      // Grab the direction data
      x = moveBindings[key][0];
      y = moveBindings[key][1];
      th = moveBindings[key][2];

    }

      // Otherwise, set the robot to stop
    else {
      x = 0;
      y = 0;
      th = 0;

    }

    // Update the Twist message
    twist.linear.x = x;
    twist.linear.y = y;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = th;

    // Publish it and resolve any remaining callbacks
    pub.publish(twist);
    ros::spinOnce();
  }

  return 0;
}


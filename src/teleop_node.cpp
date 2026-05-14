#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <map>
#include <string>
#include <vector>

const char* msg = R"(
Bipedal Teleop: Arrows + Body Control
---------------------------
Moving around:
   ↑ : forward
   ↓ : backward
   ← : turn left
   → : turn right

Body Control:
   w / s : Height Up/Down (linear.z)
   t / g : Pitch Up/Down  (angular.y)
   a / d : Roll Left/Right (angular.x)

q/z : increase/decrease max speeds by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
)";

// Format: {lin.x, lin.y, lin.z, ang.x, ang.y, ang.z}
std::map<char, std::vector<float>> moveBindings = {
    // Body Control - Height (linear.z)
    {'w', {0, 0, -1, 0, 0, 0}},
    {'s', {0, 0, 1, 0, 0, 0}},
    // Body Control - Pitch (angular.y)
    {'t', {0, 0, 0, 0,  1, 0}},
    {'g', {0, 0, 0, 0, -1, 0}},
    
    // Body Control - Roll (angular.x)
    {'a', {0, 0, 0,  1, 0, 0}},
    {'d', {0, 0, 0, -1, 0, 0}},
    
};

// Map for speed changes
std::map<char, std::vector<float>> speedBindings = {
    {'q', {1.1, 1.1}},
    {'z', {0.9, 0.9}},
    {'e', {1.0, 1.1}},
    {'c', {1.0, 0.9}},
};

class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode() : Node("teleop_node")
    {
        this->declare_parameter("stamped", false);
        this->declare_parameter("frame_id", "");
        this->declare_parameter("speed", 0.5);
        this->declare_parameter("turn", 1.0);

        stamped_ = this->get_parameter("stamped").as_bool();
        frame_id_ = this->get_parameter("frame_id").as_string();
        speed_ = this->get_parameter("speed").as_double();
        turn_ = this->get_parameter("turn").as_double();

        if (!stamped_ && !frame_id_.empty()) {
            RCLCPP_ERROR(this->get_logger(), 
                        "'frame_id' can only be set when 'stamped' is True");
            rclcpp::shutdown();
            return;
        }

        if (stamped_) {
            pub_stamped_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
        } else {
            pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        }

        setupTerminal();
        printf("%s", msg);
    }

    ~TeleopNode()
    {
        restoreTerminal();
        if (stamped_) {
            geometry_msgs::msg::TwistStamped stop_msg;
            stop_msg.header.stamp = this->now();
            stop_msg.header.frame_id = frame_id_;
            pub_stamped_->publish(stop_msg);
        } else {
            geometry_msgs::msg::Twist stop_msg;
            pub_->publish(stop_msg);
        }
    }

    void run()
    {
        float lx=0, ly=0, lz=0, ax=0, ay=0, az=0;
        int status = 0;

        while (rclcpp::ok()) {
            char key = getKey();
            if (key == '\x03') break;

            if (key == '\x1B') { // Arrow Keys
                char seq[2];
                if (read(STDIN_FILENO, &seq[0], 1) > 0 && read(STDIN_FILENO, &seq[1], 1) > 0) {
                    if (seq[0] == '[') {
                        
                        switch (seq[1]) {
                            case 'A': lx = 1;  break; // Up
                            case 'B': lx = -1; break; // Down
                            case 'C': az = -1; break; // Right
                            case 'D': az = 1;  break; // Left
                        }
                        publishTwist(lx, ly, lz, ax, ay, az);
                        continue;
                    }
                }
            }

            if (key == 'p') {
                publishTwist(0, 1.0, 0, 0, 0, 0);
                continue;
            }

            if (key == 'l') {
                publishTwist(0, -1.0, 0, 0, 0, 0);
                continue;
            }

            if (moveBindings.find(key) != moveBindings.end()) {
                lx = moveBindings[key][0];
                ly = moveBindings[key][1];
                lz = moveBindings[key][2];
                ax = moveBindings[key][3];
                ay = moveBindings[key][4];
                az = moveBindings[key][5];
            }
            else if (speedBindings.find(key) != speedBindings.end()) {
                speed_ *= speedBindings[key][0];
                turn_ *= speedBindings[key][1];
                printf("currently:\tspeed %.2f\tturn %.2f\n", speed_, turn_);
                if (status == 14) {
                    printf("%s", msg);
                }
                status = (status + 1) % 15;
                
                publishTwist(lx, ly, lz, ax, ay, az);
                continue;
            }
            else {
                lx=0; ly=0; lz=0; ax=0; ay=0; az=0;
            }

            publishTwist(lx, ly, lz, ax, ay, az);
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_stamped_;
    
    bool stamped_;
    std::string frame_id_;
    double speed_;
    double turn_;
    
    struct termios old_terminal_;

    void setupTerminal()
    {
        // Get current terminal settings
        tcgetattr(STDIN_FILENO, &old_terminal_);
        
        // Set new terminal settings
        struct termios new_terminal = old_terminal_;
        new_terminal.c_lflag &= ~(ICANON | ECHO);
        new_terminal.c_cc[VMIN] = 0;
        new_terminal.c_cc[VTIME] = 0;
        
        tcsetattr(STDIN_FILENO, TCSANOW, &new_terminal);
        
        // Set non-blocking mode
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }

    void restoreTerminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal_);
    }

    char getKey()
    {
        char key;
        while (read(STDIN_FILENO, &key, 1) != 1) {
            if (!rclcpp::ok()) {
                return '\x03';  // Return CTRL-C if node is shutting down
            }
            usleep(10000);  // Sleep for 10ms
        }
        return key;
    }

    void publishTwist(float lx, float ly, float lz, float ax, float ay, float az)
    {
        if (stamped_) {
            geometry_msgs::msg::TwistStamped twist_msg;
            twist_msg.header.stamp = this->now();
            twist_msg.header.frame_id = frame_id_;
            
            twist_msg.twist.linear.x = lx * speed_;
            twist_msg.twist.linear.y = ly * speed_;
            twist_msg.twist.linear.z = lz * speed_;
            twist_msg.twist.angular.x = ax * turn_;
            twist_msg.twist.angular.y = ay * turn_;
            twist_msg.twist.angular.z = az * turn_;
            
            pub_stamped_->publish(twist_msg);
        } else {
            geometry_msgs::msg::Twist twist_msg;
            
            twist_msg.linear.x = lx * speed_;
            twist_msg.linear.y = ly * speed_;
            twist_msg.linear.z = lz * speed_;
            twist_msg.angular.x = ax * turn_;
            twist_msg.angular.y = ay * turn_;
            twist_msg.angular.z = az * turn_;
            
            pub_->publish(twist_msg);
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
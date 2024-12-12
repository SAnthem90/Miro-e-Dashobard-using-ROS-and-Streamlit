import rospy
from geometry_msgs.msg import TwistStamped
import streamlit as st
import time

# Function to initialize ROS node once
try:
    rospy.init_node('Miro_Sensor_Dashboard', anonymous=True,disable_signals=True)
except rospy.ROSException:
    # Node is already initialized, which is fine
    pass

# Initialize the ROS Publisher
pub = rospy.Publisher('/miro/control/cmd_vel', TwistStamped, queue_size=10)

# Function to send the TwistStamped command
def send_cmd_vel(linear_x, angular_z, duration):
    # Create a TwistStamped message
    twist_stamped = TwistStamped()
    twist_stamped.header.stamp = rospy.Time.now()  # Set the timestamp to the current time
    twist_stamped.header.frame_id = "base_link"  # Or whatever frame your robot uses
    
    # Set linear and angular velocities
    twist_stamped.twist.linear.x = linear_x
    twist_stamped.twist.angular.z = angular_z
    
    # Publish the message
    for i in range(int(duration) * 10):
        pub.publish(twist_stamped)
        time.sleep(0.1)

# Streamlit Interface to control the robot with virtual buttons
def control_with_buttons():
    # Default velocities (initial values)
    d = 0
    linear_x = st.session_state.get("linear_x", 0.0)
    angular_z = st.session_state.get("angular_z", 0.0)

    st.write("# Miro-e Dashboard for Controlling Velocities 🏎️💨")
    col1, col2, col3 = st.columns(3)
    st.subheader("Control Using Buttons")
    with col1:
        forward_button = st.button("↑ Forward")
        backward_button = st.button("↓ Backward")
    with col2:
        left_button = st.button("← Left")
        right_button = st.button("→ Right")  
    
    duration = st.text_input("Enter the Duration", value=str(d), key=123)
    
    # Control robot with virtual buttons
    if forward_button:
        linear_x = 0.5
        angular_z = 0.0
        st.success(f"Moving forward for {duration} seconds")
    elif backward_button:
        linear_x = -0.5
        angular_z = 0.0
        st.success(f"Moving backward for {duration} seconds")
    elif left_button:
        linear_x = 0.0
        angular_z = 0.5
        st.success(f"Turning left for {duration} seconds")
    elif right_button:
        linear_x = 0.0
        angular_z = -0.5
        st.success(f"Turning right for {duration} seconds")
    
    # Stop the robot if no buttons are pressed
    if not (forward_button or backward_button or left_button or right_button):
        linear_x = 0.0
        angular_z = 0.0

    # Store the updated velocities in session state for future use
    st.session_state.linear_x = linear_x
    st.session_state.angular_z = angular_z

    # Send the robot command for current velocities (from buttons)
    send_cmd_vel(linear_x, angular_z, duration)

    # Display current velocities from the virtual buttons
    with col3:
        st.write("Linear Velocity (m/s): ", linear_x)
        st.write("Angular Velocity (rad/s): ", angular_z)

# Streamlit Interface to control the robot with custom velocity inputs
def control_with_custom_velocity():
    st.subheader("Custom Velocity Control")
    d = 0
    # Default velocities (from session state)
    linear_x = st.session_state.get("linear_x", 0.0)
    angular_z = st.session_state.get("angular_z", 0.0)

    linear_x_input = st.text_input("Enter Linear Velocity (m/s)", value=str(linear_x))
    angular_z_input = st.text_input("Enter Angular Velocity (rad/s)", value=str(angular_z))
    duration = st.text_input("Enter the Duration", value=str(d), key=321)

    # Add a button to "send" the custom velocities
    if st.button("Send"):
        # Ensure the inputs are valid floats
        try:
            linear_x_input = float(linear_x_input)
            angular_z_input = float(angular_z_input)
            # Send the command to the robot
            send_cmd_vel(linear_x_input, angular_z_input, duration)
            st.success(f"Sent: Linear Velocity = {linear_x_input} m/s, Angular Velocity = {angular_z_input} rad/s for {duration} seconds")
        except ValueError:
            st.error("Please enter valid numeric values for velocity.")
    
    # Store the custom velocities in session state for future use
    st.session_state.linear_x = linear_x_input
    st.session_state.angular_z = angular_z_input

# Streamlit Interface
def control_robot():
    # Initialize ROS Node once


    # Control robot using virtual buttons and custom velocity inputs
    control_with_buttons()
    control_with_custom_velocity()

if __name__ == "__main__":
    control_robot()

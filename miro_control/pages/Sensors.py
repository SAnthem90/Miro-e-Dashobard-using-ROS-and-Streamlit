#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, Int16MultiArray, UInt16
from sensor_msgs.msg import Range, JointState, Imu, CompressedImage
from nav_msgs.msg import Odometry
import streamlit as st
import threading
import time
import numpy as np
import cv2
from cv_bridge import CvBridge

# Global variables to store latest sensor data
latest_distance = None
latest_light = None
latest_mic = None
latest_body_touch = None
latest_head_touch = None
latest_kinematic_joints = None  
# Global variables for IMU sensor data
latest_imu_body = None
latest_imu_head = None

# New sensor variables
latest_wheel_speed_cmd = None
latest_wheel_speed_opto = None
latest_wheel_speed_back_emf = None
latest_wheel_effort_pwm = None
latest_odom = None
latest_left_camera = None
latest_right_camera = None

# Initialize CvBridge for converting ROS images to OpenCV format
bridge = CvBridge()

# Set up page configuration for better layout
st.set_page_config(
    page_title="Miro-e Robot Sensors",
    page_icon=":robot:",
    layout="wide"
)

# Main display function
def main_display():
    # Create a title with some styling
    st.markdown("""
    <h1 style='text-align: center; color: #3a5a8f;'>
    🤖 Miro Robot Sensors 
    </h1>
    """, unsafe_allow_html=True)

    # Create four columns for sensors (including kinematic joints and cameras)
    col1, col2, col3, col4 = st.columns(4)
    # Sonar Distance Column
    with col1:
        st.markdown("### 📏 Sonar Distance")
        sonar_placeholder = st.empty()
        st.markdown("### 🦾 Body Touch Sensor")
        body_touch_holder = st.empty()
        st.markdown("### 🤖 Head Touch Sensor")
        head_touch_holder = st.empty()
        st.markdown("### 🎤 Microphone Data")
        mic_placeholder = st.empty()
        st.success("ROS Active")

    # Light Sensors Column
    with col2:
        st.markdown("### 💡 Light Intensities")
        light_placeholder = st.empty()
        st.markdown("### 🦾 Kinematic Joints (Rad)")
        kinematic_placeholder = st.empty()
        st.markdown("### 🦾 IMU Body Acceleration (m/s²)")
        imu_body_placeholder = st.empty()
        st.markdown("### 🤖 IMU Head Acceleration (m/s²)")
        imu_head_placeholder = st.empty()

    # Wheel and Odometry Column
    with col3:
        st.markdown("### 🛞 Wheel Speed Command (m/s)")
        wheel_speed_cmd_placeholder = st.empty()
        st.markdown("### 🛞 Wheel Speed Opto (m/s)")
        wheel_speed_opto_placeholder = st.empty()
        st.markdown("### 🛞 Wheel Speed Back EMF (m/s)")
        wheel_speed_back_emf_placeholder = st.empty()
        st.markdown("### ⚡ Wheel Effort PWM")
        wheel_effort_pwm_placeholder = st.empty()
        st.markdown("### 🧭 Odometry")
        odom_placeholder = st.empty()

    # Camera Column (New column for camera images)
    with col4:
        st.markdown("### 📷 Left Camera")
        left_camera_placeholder = st.empty()
        st.markdown("### 📷 Right Camera")
        right_camera_placeholder = st.empty()

    # Callback functions
    def sonar_callback(data):
        global latest_distance
        latest_distance = round(data.range, 2)

    def light_sensors_callback(data):
        global latest_light
        lights = list(data.data)
        lights_labels = ['Front Left', 'Front Right', 'Rear Left', 'Rear Right']
        latest_light = [
            f"{label}: {round(value, 2)}" 
            for label, value in zip(lights_labels, lights)
        ]

    def mic_callback(data):
        global latest_mic
        latest_mic = max(np.array(data.data))
    
    def body_callback(data):
        global latest_body_touch
        latest_body_touch = data.data
    
    def head_callback(data):
        global latest_head_touch
        latest_head_touch = data.data

    # New callback function for kinematic joints data
    def kinematic_callback(data):
        global latest_kinematic_joints
        # Extract and format joint positions: TILT, LIFT, YAW, PITCH
        latest_kinematic_joints = {
            "TILT": round(data.position[0], 2),
            "LIFT": round(data.position[1], 2),
            "YAW": round(data.position[2], 2),
            "PITCH": round(data.position[3], 2)
        }

    def imu_body_callback(data):
        global latest_imu_body
        latest_imu_body = {
            "x": round(data.linear_acceleration.x, 2),
            "y": round(data.linear_acceleration.y, 2),
            "z": round(data.linear_acceleration.z, 2)
        }

    # IMU Head Callback
    def imu_head_callback(data):
        global latest_imu_head
        latest_imu_head = {
            "x": round(data.linear_acceleration.x, 2),
            "y": round(data.linear_acceleration.y, 2),
            "z": round(data.linear_acceleration.z, 2)
        }

    # New sensor callbacks
    def wheel_speed_cmd_callback(data):
        global latest_wheel_speed_cmd
        latest_wheel_speed_cmd = list(data.data)

    def wheel_speed_opto_callback(data):
        global latest_wheel_speed_opto
        latest_wheel_speed_opto = list(data.data)

    def wheel_speed_back_emf_callback(data):
        global latest_wheel_speed_back_emf
        latest_wheel_speed_back_emf = list(data.data)

    def wheel_effort_pwm_callback(data):
        global latest_wheel_effort_pwm
        latest_wheel_effort_pwm = list(data.data)

    def odom_callback(data):
        global latest_odom
        # Extract linear and angular velocities from the odometry message
        latest_odom = {
            "linear_x": round(data.twist.twist.linear.x, 2),
            "angular_z": round(data.twist.twist.angular.z, 2)
        }

    # New camera callbacks
    def left_camera_callback(data):
        global latest_left_camera
        try:
            # Convert the ROS image message to OpenCV format and then to a Streamlit displayable format
            latest_left_camera = bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception as e:
            print(f"Error converting left camera image: {e}")

    def right_camera_callback(data):
        global latest_right_camera
        try:
            # Convert the ROS image message to OpenCV format and then to a Streamlit displayable format
            latest_right_camera = bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception as e:
            print(f"Error converting right camera image: {e}")

    # ROS Node setup function
    def setup_ros_node():
        rospy.init_node("Miro_Sensor_Dashboard", disable_signals=True,anonymous=True)
        rospy.Subscriber("/miro/sensors/sonar", Range, sonar_callback)
        rospy.Subscriber("/miro/sensors/light", Float32MultiArray, light_sensors_callback)
        rospy.Subscriber("/miro/sensors/mics", Int16MultiArray, mic_callback)
        rospy.Subscriber("/miro/sensors/touch_body", UInt16, body_callback)
        rospy.Subscriber("/miro/sensors/touch_head", UInt16, head_callback)
        rospy.Subscriber("/miro/sensors/kinematic_joints", JointState, kinematic_callback)  # New subscription
        rospy.Subscriber("/miro/sensors/imu_body", Imu, imu_body_callback)  # New IMU Body subscription
        rospy.Subscriber("/miro/sensors/imu_head", Imu, imu_head_callback)  # New IMU Head subscription
        rospy.Subscriber("/miro/sensors/wheel_speed_cmd", Float32MultiArray, wheel_speed_cmd_callback)  # Wheel Speed Cmd
        rospy.Subscriber("/miro/sensors/wheel_speed_opto", Float32MultiArray, wheel_speed_opto_callback)  # Wheel Speed Opto
        rospy.Subscriber("/miro/sensors/wheel_speed_back_emf", Float32MultiArray, wheel_speed_back_emf_callback)  # Wheel Speed Back EMF
        rospy.Subscriber("/miro/sensors/wheel_effort_pwm", Float32MultiArray, wheel_effort_pwm_callback)  # Wheel Effort PWM
        rospy.Subscriber("/miro/sensors/odom", Odometry, odom_callback)  # Odometry subscription
        rospy.Subscriber("/miro/sensors/caml/compressed", CompressedImage, left_camera_callback)  # Left camera subscription
        rospy.Subscriber("/miro/sensors/camr/compressed", CompressedImage, right_camera_callback)  # Right camera subscription
        rospy.spin()

    # Update display loop
    def update_display():
        while True:
            try:
                # Update Sonar Distance
                if latest_distance is not None:
                    sonar_placeholder.metric(
                        label="Distance", 
                        value=f"{latest_distance} m",
                        help="Obstacle detection distance"
                    )

                # Update Light Intensities
                if latest_light is not None:
                    light_text = "\n".join(latest_light)
                    light_placeholder.code(light_text)

                # Update Microphone Data
                if latest_mic is not None:
                    mic_placeholder.metric(
                        label="Max Intensity", 
                        value=str(latest_mic),
                        help="Maximum microphone intensity"
                    )

                # Update Body and Head Touch Sensors
                if latest_body_touch is not None:
                    body_touch_holder.metric(label="Body Touch Sensor Value", value=latest_body_touch)

                if latest_head_touch is not None:
                    head_touch_holder.metric(label="Head Touch Sensor Value", value=latest_head_touch)

                # Update Kinematic Joints
                if latest_kinematic_joints is not None:
                    kinematic_text = "\n".join([f"{key}: {value} rad" for key, value in latest_kinematic_joints.items()])
                    kinematic_placeholder.code(kinematic_text)

                # Update IMU Body Acceleration
                if latest_imu_body is not None:
                    imu_body_text = (
                        f"X: {latest_imu_body['x']} m/s²\n"
                        f"Y: {latest_imu_body['y']} m/s²\n"
                        f"Z: {latest_imu_body['z']} m/s²"
                    )
                    imu_body_placeholder.code(imu_body_text)

                # Update IMU Head Acceleration
                if latest_imu_head is not None:
                    imu_head_text = (
                        f"X: {latest_imu_head['x']} m/s²\n"
                        f"Y: {latest_imu_head['y']} m/s²\n"
                        f"Z: {latest_imu_head['z']} m/s²"
                    )
                    imu_head_placeholder.code(imu_head_text)

                # Update Wheel Speed Command
                if latest_wheel_speed_cmd is not None:
                    wheel_speed_cmd_placeholder.code("\n".join([f"Wheel {i+1}: {speed} m/s" for i, speed in enumerate(latest_wheel_speed_cmd)]))

                # Update Wheel Speed Opto
                if latest_wheel_speed_opto is not None:
                    wheel_speed_opto_placeholder.code("\n".join([f"Wheel {i+1}: {speed} m/s" for i, speed in enumerate(latest_wheel_speed_opto)]))

                # Update Wheel Speed Back EMF
                if latest_wheel_speed_back_emf is not None:
                    wheel_speed_back_emf_placeholder.code("\n".join([f"Wheel {i+1}: {speed} m/s" for i, speed in enumerate(latest_wheel_speed_back_emf)]))

                # Update Wheel Effort PWM
                if latest_wheel_effort_pwm is not None:
                    wheel_effort_pwm_placeholder.code("\n".join([f"Wheel {i+1}: {pwm}" for i, pwm in enumerate(latest_wheel_effort_pwm)]))

                # Update Odometry
                if latest_odom is not None:
                    odom_text = (
                        f"Linear X: {latest_odom['linear_x']} m/s\n"
                        f"Angular Z: {latest_odom['angular_z']} rad/s"
                    )
                    odom_placeholder.code(odom_text)

                # Update Camera Images
                if latest_left_camera is not None:
                    left_camera_placeholder.image(latest_left_camera, channels="BGR", caption="Left Camera", use_container_width=True)

                if latest_right_camera is not None:
                    right_camera_placeholder.image(latest_right_camera, channels="BGR", caption="Right Camera", use_container_width=True)

                time.sleep(0.1)  # Smooth update rate

            except Exception as e:
                st.error(f"Display update error: {e}")
                time.sleep(1)

    # Start ROS node in a separate thread
    ros_thread = threading.Thread(target=setup_ros_node, daemon=True)
    ros_thread.start()

    # Start display update
    update_display()

# Main execution
if __name__ == '__main__':
    main_display()

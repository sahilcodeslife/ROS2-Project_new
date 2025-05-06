import cv2

# Read image
img = cv2.imread('/home/rosdev/ros2_ws/src/camera/camera/hard.PNG')

# Convert to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Save or display
cv2.imwrite('/home/rosdev/ros2_ws/src/camera/camera/hard_grey.PNG', gray)

import cv2
import easyocr

# Instantiate the text detector
reader = easyocr.Reader(['en'], gpu=False)

# Open the default camera (you may need to change the camera index)
cap = cv2.VideoCapture(0)

# Set the threshold for text detection
threshold = 0.1  # Adjust the threshold as needed

while True:
    # Read a frame from the camera
    ret, frame = cap.read()
    if not ret:
        break

    # Detect text on the frame
    text_ = reader.readtext(frame)

    letter_detected = False  # Flag to check if 'R' is detected

    # Draw bounding boxes and text
    for t_, t in enumerate(text_):
        bbox, text, score = t

        if score > threshold:
            # Convert bounding box coordinates to integers
            bbox = [(int(coord[0]), int(coord[1])) for coord in bbox]

            # Draw the rectangle
            cv2.rectangle(frame, bbox[0], bbox[2], (0, 255, 0), 5)
            cv2.putText(frame, text, bbox[0], cv2.FONT_HERSHEY_COMPLEX, 0.65, (255, 0, 0), 2)

            # Check if 'R' is detected in the text
            if 'R' in text:
                print('Right')
            elif 'L' in text:
                print('Left')


            # Print the detected text and score for debugging
            print(f"Detected Text: {text}, Score: {score}")

    cv2.imshow('Text Detection', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()


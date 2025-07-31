import cv2
import os
from datetime import datetime

device_range = range(4, 10)

print("Each video device will be tested sequentially. Press 'q' to move to the next device, 's' to save a picture, 'r' to start/stop recording.")

for i in device_range:
    path = f"/dev/video{i}"
    cap = cv2.VideoCapture(path)

    if not cap.isOpened():
        print(f"{path} could not be opened.")
        continue

    print(f"{path} opened. Check if video is coming. (Press q to exit, s to save picture, r to start/stop recording)")
    
    # Video recording variables
    video_writer = None
    recording = False
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"{path}: frame could not be captured.")
            break
        cv2.imshow(f"Camera: {path}", frame)
        
        # Write frame to video if recording
        if recording and video_writer is not None:
            video_writer.write(frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            # Create directory if it doesn't exist
            if not os.path.exists('saved_images'):
                os.makedirs('saved_images')
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"saved_images/camera_{i}_{timestamp}.jpg"
            
            # Save the image
            cv2.imwrite(filename, frame)
            print(f"Picture saved as: {filename}")
        elif key == ord('r'):
            if not recording:
                # Start recording
                if not os.path.exists('saved_videos'):
                    os.makedirs('saved_videos')
                
                # Generate video filename with timestamp
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                video_filename = f"saved_videos/camera_{i}_{timestamp}.mp4"
                
                # Get frame dimensions
                height, width, _ = frame.shape
                
                # Initialize video writer with MP4 and H.264 codec
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                video_writer = cv2.VideoWriter(video_filename, fourcc, 20.0, (width, height))
                
                recording = True
                print(f"Started recording: {video_filename}")
            else:
                # Stop recording
                recording = False
                if video_writer is not None:
                    video_writer.release()
                    video_writer = None
                print("Recording stopped")

    cap.release()
    cv2.destroyAllWindows()
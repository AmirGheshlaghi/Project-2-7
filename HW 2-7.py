import cv2
import numpy as np
from ultralytics import YOLO

model = YOLO("models/yolov8x")
traffic= cv2.VideoCapture("traffic.mp4")

i = 0
car_center = []
motor_center = []
counter_car = 0
counter_motor = 0
detect_line = 300

while True:

	car_center_new = []
	motor_center_new = []
	dist_car = []
	dist_motor = []

	ret_video, frame_video = traffic.read()

	frame_video = cv2.resize(frame_video, (800, 450))
	frame_video_show = frame_video.copy()

	# Determining the detection range
	frame_video = frame_video[detect_line:, :, :]

	if ret_video:

		results = model.predict(source=frame_video)

		# Draw detection line
		cv2.line(frame_video_show, (70, detect_line), (565, detect_line), (0, 0, 255), 2)

		for result in results[0].boxes.boxes:
			if (int(result[-1].item()) == int(2) or int(result[-1].item()) == int(3)):

				x1 = result[0].item()
				y1 = result[1].item() + detect_line
				x2 = result[2].item()
				y2 = result[3].item() + detect_line

				if int(result[-1].item()) == int(2):

					# Draw detected cars
					cv2.rectangle(frame_video_show, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)

					if i == 0:

						# Counting cars in the first frame
						counter_car += 1 

				else:

					# Draw detected motorcycles
					cv2.rectangle(frame_video_show, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
					if i == 0:

						# Counting motorcycles in the first frame
						counter_motor += 1

				if int(result[-1].item()) == int(2):

					# Determining the coordinates of the center of the machines
					car_center_new.append([(int(result[0].item()) + int(result[2].item()))/2, (int(result[1].item()) \
						+ int(result[3].item()))/2])

				else:

					# Determining the coordinates of the center of the motorcycles
					motor_center_new.append([(int(result[0].item()) + int(result[2].item()))/2, (int(result[1].item()) + \
					 int(result[3].item()))/2])

		if i != 0:

			for cent_car_new in car_center_new:
				for cent_car in car_center:

					# Determining the distance of the cars in the new frame compared to the previous frame
					dist_car.append((((cent_car_new[0] - cent_car[0])**2 + (cent_car_new[1] - cent_car[1])**2)**0.5))

			# Counting cars
			if len(dist_car) != 0 and sum(x < 30 for x in dist_car) != len(car_center_new):
				counter_car += 1 

			elif len(car_center) == 0 and len(car_center_new) != 0:
				counter_car += 1 

			for cent_motor_new in motor_center_new:
				for cent_motor in motor_center:

					# Determining the distance of the motorcycles in the new frame compared to the previous frame
					dist_motor.append((((cent_motor_new[0] - cent_motor[0])**2 + (cent_motor_new[1] - cent_motor[1])**2)**0.5))

			# Counting motorcycles
			if len(dist_motor) != 0 and sum(x < 30 for x in dist_motor) != len(motor_center_new):
				counter_motor += 1

			elif len(motor_center) == 0 and len(motor_center_new) != 0:
				counter_motor += 1

		# Display the number of cars and motorcycles
		cv2.putText(frame_video_show, "car: " + f"{counter_car}", (50, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 100, 0), 2)
		cv2.putText(frame_video_show, "motor: " + f"{counter_motor}", (150, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 100, 0), 2)
		
		car_center = car_center_new
		motor_center = motor_center_new

		cv2.imshow("traffic" ,frame_video_show)

		q = cv2.waitKey(0)
		if q == ord('q'):
			break
	else:
		break

	i += 1

trrafic.release()
cv2.destroyAllWindows()
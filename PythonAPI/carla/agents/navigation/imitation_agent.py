
import torch
import argparse
import numpy as np
from PIL import Image
import pytorch_lightning as pl
import carla
from agents.navigation.imitation_learning.imitation_network import ImitationNetwork
from agents.navigation.basic_agent import BasicAgent
import cv2
import logging

# logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)

class ImitationAgent(BasicAgent):

	def __init__(self, vehicle, image_cut=[115, 510]):
		BasicAgent.__init__(self, vehicle)
		self.image_cut = image_cut
		hparams = argparse.Namespace(**{'learning_rate':1,
			'train_batch_size': 1, 'val_batch_size': 1})
		self.cil_net = ImitationNetwork.load_from_checkpoint("/home/jacopobartiromo/cil_carla/data-and-checkpoints/model_checkpoints/cil-epoch=57-train_loss=6.10.ckpt")
		# self.cil_net.freeze()
		self.input_image_size = (200, 88)

	def run_step(self, speed, direction, image):
		control = self.compute_action(speed, direction, image)
		return control

	def compute_action(self, speed, direction, image):
		# rgb_image = rgb_image #[self.image_cut[0]:self.image_cut[1], :]
		# frame = cv2.imread("/home/jacopobartiromo/cil_carla/current_image.jpg")
		rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		# cv2.imshow("image", image)
		# cv2.waitKey()
		image_input = np.array(Image.fromarray(rgb_image).resize(size=self.input_image_size))
		image_input = image_input.astype(np.float32)
		image_input = np.multiply(image_input, 1.0 / 255.0)
		print(image_input[0][0])
		tsteer, tacc, tbrake = self.control_function(image_input, speed, direction)
		steer = tsteer.item()
		acc = tacc.item()
		brake = tbrake.item()

		print("steer: "+str(steer)+" acc: " + str(acc) + " brake: " + str(brake))

		if brake < 0.1:
			brake = 0.0

		if acc > brake:
			brake = 0.0

		if speed > 5.0 and brake == 0.0:
			acc = 0.0

		control = carla.VehicleControl()
		control.steer = steer
		control.throttle = acc
		control.brake = brake
		control.hand_brake = 0
		control.reverse = 0
		return control

	def control_function(self, image_input, speed, direction):
		image = torch.tensor(np.array(image_input)).unsqueeze(dim=0).type(torch.FloatTensor)
		vel = torch.tensor(np.array(speed)).unsqueeze(dim=0).type(torch.FloatTensor)
		control = torch.tensor(np.array(direction)).unsqueeze(dim=0).type(torch.FloatTensor)
		input_data = [image, vel, control]
		output = self.cil_net(input_data)
		steer = output[0][0]
		acc = output[0][1]
		brake = output[0][2]
		return steer, acc, abs(brake)

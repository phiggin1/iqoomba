#!/usr/bin/env python3

import rospy
from subprocess import run, PIPE

DATA_PATH = "/home/phiggin1/deep_rgbd_v01.2/data/"
CAFFE_PATH = "/home/phiggin1/deep_rgbd_v01.2/rgbdnet_caffe/build/tools/"
MODEL_PATH = "/home/phiggin1/deep_rgbd_v01.2/caffe_models/"

def get_features():
	output = run([
		"rm",
		"-rf",
		DATA_PATH+"/fus/testblob_lmdb"
	], stdout=PIPE, stderr=PIPE)


	output = run([
		CAFFE_PATH+"/convert_imageset_multi.bin",
		"-gray",
		"/",
		DATA_PATH+"/fus/testlist.txt",
		DATA_PATH+"/fus/testblob_lmdb/",
		"256"
	], stdout=PIPE, stderr=PIPE)

	output = run([
		CAFFE_PATH+"/caffe.bin", 
		"test", 
		"-model",MODEL_PATH+"/train_val-fus.prototxt", 
		"-weights",MODEL_PATH+"/s00_fus_rgb_depth_jet.caffemodel", 
		"-gpu", "0", 
		"-iterations", "1"
	], stdout=PIPE, stderr=PIPE)

	lines =  str(output.stderr, 'utf-8').split('\n')

	features = []

	for l in lines[-53:-2]:
		s = l.split('=')
		features.append(float(s[1])

	print(features)

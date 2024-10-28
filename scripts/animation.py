#!/usr/bin/env python
# Copyright (c) 2013-2018 Hanson Robotics, Ltd, all rights reserved 

import time
import random
import logging
import json
import numpy as np

import rospy
from hr_msgs.msg import SetExpression, SetAnimation
from r2_behavior.cfg import AnimationConfig
from dynamic_reconfigure.server import Server
logger = logging.getLogger('hr.r2_behavior.animation')


class Utils:
    def __init__(self):
        pass

    @staticmethod
    # Picks random expressions gesture from multidimensional array which has probabilities in first column.
    # Returns row of array
    def random(entries):
        probabilities_sum = sum([e['probability'] for e in entries])
        probabilities = [e['probability'] / probabilities_sum for e in entries]
        i = np.random.choice(len(probabilities), p=probabilities)
        return entries[i]


class Expressions:
    def __init__(self, config, publisher):
        self.publisher = publisher
        self.config = config
        self.next_expression_time = 0
        self.smile = 0
        # Future time
        self.smile_started = time.time()*2

    def show_expression(self):
        t = time.time()

        if t > self.next_expression_time:
            self.smile_started = time.time()*2
            self.next_expression_time = t + random.uniform(self.config['time_between_expressions_min'],
                                                           self.config['time_between_expressions_max'])
            try:
                e = Utils.random(json.loads(self.config['expressions']))
            except Exception as exc:
                return
            expression = SetExpression()
            expression.name = e['name']
            expression.magnitude = max(0, min(1, random.uniform(float(e['magnitude_min']), float(e['magnitude_max'])) *
                                              self.config['expression_magnitude']))
            expression.duration = rospy.Duration(
                max(1, random.uniform(float(e['duration_min']), float(e['duration_max'])) *
                    self.config['expression_duration']))
            logger.info('animation-expression', extra={'data': {
                'name': expression.name,
                'magnitude': expression.magnitude,
                'duration': expression.duration.secs}})
            self.publisher.publish(expression)
        return None


class Gestures:
    def __init__(self, config, publisher):
        self.config = config
        self.next_gesture_time = 0
        self.publisher = publisher
        self.current_gesture = None
        self.current_gesture_end = 0.0

    def update_cfg(self, config):
        # Check if animations need to be stopped
        if self.config.stop_running_gesture_on_update and self.current_gesture is not None:
            # If more than 1 second left 
            if self.current_gesture_end > time.time() + 1.0:
                self.current_gesture.magnitude = -1.0
                self.publisher.publish(self.current_gesture)
        # Reset current gesture
        self.current_gesture = None
        self.config = config
        try:
            if config.trigger_gesture_on_update:
                self.next_gesture_time = 0
        except Exception: 
            pass

    def show_gesture(self):
        t = time.time()
        if t > self.next_gesture_time:
            self.next_gesture_time = t + random.uniform(self.config['time_between_gestures_min'],
                                                       self.config['time_between_gestures_max'])
            try:
                g = Utils.random(json.loads(self.config['gestures']))
            except:
                return
            gesture_length = rospy.get_param(f'/hr/animation/library/animations/{g["name"]}/length',2.0)
            self.next_gesture_time = t + gesture_length + random.uniform(self.config['time_between_gestures_min'],
                                                       self.config['time_between_gestures_max'])
            gesture = SetAnimation()
            gesture.name = g['name']
            gesture.magnitude = max(0, min(3, random.uniform(float(g['magnitude_min']), float(g['magnitude_max'])) *
                                           self.config['gesture_magnitude']))
            gesture.speed = min(3, random.uniform(float(g['speed_min']), float(g['speed_max'])) *
                                self.config['gesture_speed'])
            logger.info('animation-gesture', extra={'data': {
                'name': gesture.name,
                'magnitude': gesture.magnitude,
                'speed': gesture.speed}})
            self.publisher.publish(gesture)
            self.current_gesture = gesture
            self.current_gesture_end = t + gesture_length
        return None


class Animations:
    def __init__(self):
        self.config = {}
        self.init = True
        self.gestures = None
        self.expressions = None
        self.config = None
        self.expresion_pub = rospy.Publisher('/hr/animation/set_expression', SetExpression,queue_size=10)
        self.gesture_pub = rospy.Publisher('/hr/animation/set_animation', SetAnimation, queue_size=10)
        self.animations = Server(AnimationConfig, self.config_callback, namespace='/hr/behavior/current/animations')

    def config_callback(self, config, level):
        self.config = config
        if self.init:
            self.gestures = Gestures(config, self.gesture_pub )
            self.expressions = Expressions(config, self.expresion_pub)
            self.init = False
        else:
            self.gestures.update_cfg(config)
            self.expressions.config = config
        return config

    def timer(self):
        if self.init:
            return
        if not self.config.enable_flag:
            return
        # Shows gestures based on timings setr in configs
        self.expressions.show_expression()
        self.gestures.show_gesture()


if __name__ == "__main__":
    rospy.init_node("animations")
    rate = rospy.Rate(10)
    animations = Animations()
    while not rospy.is_shutdown():
        animations.timer()
        rate.sleep()



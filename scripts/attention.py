#!/usr/bin/env python
# Copyright (c) 2013-2018 Hanson Robotics, Ltd, all rights reserved
import logging
import math
import random
import threading
import time
import numpy as np
import rospy
import tf
# Attention regions
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Point, PointStamped
from hr_msgs.msg import Target, SetExpression, SetAnimation, People, TargetPosture

#from performances.nodes import attention as AttentionRegion
from r2_behavior.cfg import AttentionConfig, MirroringConfig
from std_msgs.msg import String, UInt8, Int64, Int32, Float32
from tf.transformations import quaternion_from_euler, euler_from_quaternion
logger = logging.getLogger('hr.behavior.attention')



class Ema(object):
    def __init__(self, start=0, window=10, smooth=0.5, trend_smooth=0.5, max_history=500):
        self.start = start
        self.window = float(window)
        self.value = float(start)
        self.smooth = float(smooth)
        self.trend = 0
        self.trend_smooth = float(trend_smooth)
        self.samples = 0
        self.values = []
        self.times = []
        self.max_history = max_history
        self.time_lock = threading.Lock()

    def update(self, v, s=0.5):
        self.samples += 1
        self.smooth = s
        # TODO: TBC
        self.trend_smooth = self.smooth
        self.update_value(v)
        with self.time_lock:
            self.values.append(self.value)
            self.times.append(time.time())
            self.values = self.values[-self.max_history:]
            self.times = self.times[-self.max_history:]

    def update_value(self, v):
        self.simple_exponential_smoothing(v)
        # self.double_exponential_smoothing(v)
        # self.value = v * (self.smooth / (1 + self.window)) + self.value * (1 - self.smooth / (1 + self.window))

    def simple_exponential_smoothing(self, v):
        self.value = self.smooth * v + (1 - self.smooth) * self.value if self.values else v

    def double_exponential_smoothing(self, v):
        if not self.value:
            self.value = v
        elif len(self.values) == 1:
            self.trend = v - self.value
            self.simple_exponential_smoothing(v)
        else:
            new_value = self.smooth * v + (1 - self.smooth) * (self.value + self.trend)
            self.trend = self.trend_smooth * (new_value - self.value) + (1 - self.trend_smooth) * self.trend
            self.value = new_value

    def reset(self):
        self.samples = 0
        self.value = self.start
        self.values = []
        self.times = []

    def get(self,at= None, dt=0.1):
        if at is None:
            return self.value
        else:
            found = -1
            with self.time_lock:
                for i, t in enumerate(reversed(self.times)):
                    if t < at:
                        #print("find {} looking for {}".format(t, at))
                        if dt > at - t:
                            found = len(self.times)-1-i
                        break
                if found > -1:
                    return self.values[found]
            return None
            # find value over time



class Mirroring(object):
    def __init__(self):
        self.config = None
        self.lip_corner_puller = Ema()
        self.brows = Ema()
        self.head = Ema()
        self.jaw_drop = Ema()
        self.lip_corner_depressor = Ema()
        self.upper_lip_raiser = Ema()
        self.lip_tightener = Ema()
        self.lip_stretcher = Ema()
        self.lips_part = Ema()
        self.chin_raiser = Ema()
        self.upper_lid_raiser = Ema()
        self.lid_tightener = Ema()
        self.cheek_raiser = Ema()
        self.roll = Ema()
        self.pitch = Ema()
        self.yaw = Ema()

        self.last_blink = 0
        self.min_time_between_blinks = 0.3
        self.blinks = []
        self.blink_anim = SetAnimation()
        self.blink_anim.magnitude = 1.0
        self.blink_anim.speed = 1.0

        self.expressing = 0
        self.expression_interval = 1
        self.expression_magnitude = 0
        self.neutral_exp = SetExpression()
        self.neutral_exp.name = 'happy'
        self.neutral_exp.magnitude = 0
        self.neutral_exp.duration.secs = 1
        self.anger = Ema()
        self.anger_exp = SetExpression()
        self.anger_exp.duration.secs = 2
        self.disgust = Ema()
        self.disgust_exp = SetExpression()
        self.disgust_exp.duration.secs = 2
        self.fear = Ema()
        self.fear_exp = SetExpression()
        self.fear_exp.duration.secs = 2
        self.happiness = Ema()
        self.happiness_exp = SetExpression()
        self.happiness_exp.duration.secs = 3
        self.sadness = Ema()
        self.sadness_exp = SetExpression()
        self.sadness_exp.duration.secs = 10
        self.surprise = Ema()
        self.surprise_exp = SetExpression()
        self.surprise_exp.duration.secs = 2

        self.expression_pub = rospy.Publisher('/hr/animation/set_expression', SetExpression, queue_size=1)
        # for rest could use external animation
        self.external_pub = rospy.Publisher('/hr/animation/set_state', TargetPosture, queue_size=1)
        # Blinking is better with animation
        self.animation_pub = rospy.Publisher('/hr/animation/set_animation', SetAnimation, queue_size=1)
        self.config_server = Server(MirroringConfig, self.configs_cb, namespace='/hr/behavior/mirroring')

    def update_faces(self, person_msg):
        if not self.config.enabled:
            return
        aus_msg = person_msg.face.action_units
        aus = {au.name: au.intensity * au.presence for au in aus_msg}
        q = person_msg.face.head_pose.orientation
        roll = 0
        pitch = 0
        yaw = 0
        if  abs(q.w) > 0.01:
            e = euler_from_quaternion([q.x,q.y,q.z,q.w])
            roll = math.pi + e[2] if e[2] < 0 else e[2] - math.pi
            pitch = math.pi + e[0] if e[0] < 0 else e[0] - math.pi
            yaw = e[1]
            #Filter big values
            if abs(roll) > math.pi/4.0:
                roll = 0
            if abs(pitch) > math.pi/4.0:
                pitch = 0
            if abs(yaw) > math.pi/4.0:
                yaw = 0
        self.roll.update(roll, self.config.mirror_head_roll_smooth)
        self.pitch.update(pitch, self.config.mirror_head_pitch_smooth)
        self.yaw.update(yaw, self.config.mirror_head_yaw_smooth)
        if len(aus):
            self.brows.update((aus['OuterBrowRaise'] - aus['BrowLower']) / 2.0, self.config.mirror_brows_smooth)
            self.lip_corner_puller.update(aus['Smile'], self.config.mirror_lip_corner_puller_smooth)
            self.jaw_drop.update(aus['Jaw'], self.config.mirror_jaw_drop_smooth)
            if aus['Blink'] > 0.15:
                if time.time() > self.last_blink + self.min_time_between_blinks:
                    self.blinks.append(time.time())
                    self.blinks = self.blinks[-20:]
                # if eyes are closed keep updating the blinking time so it waits for eyes to open
                self.last_blink = time.time()
            self.lip_corner_depressor.update(aus['Frown'], self.config.mirror_lip_corner_depressor_smooth)
            self.upper_lip_raiser.update(aus['UpperLipRaise'], self.config.mirror_upper_lip_raiser_smooth)
            self.lip_tightener.update(aus['LipFunnel'], self.config.mirror_lip_tightener_smooth)
            self.lip_stretcher.update(aus['LipStretch'], self.config.mirror_lip_stretcher_smooth)
            self.lips_part.update(aus['LipsApart'], self.config.mirror_lips_part_smooth)
            self.chin_raiser.update(aus['ChinRaise'], self.config.mirror_chin_raiser_smooth)
            self.upper_lid_raiser.update(aus['UpperLidRaise'], self.config.mirror_upper_lid_raiser_smooth)
            self.lid_tightener.update(aus['LidTighten'], self.config.mirror_lid_tightener_smooth)
            self.cheek_raiser.update(aus['CheeckRaise'], self.config.mirror_cheek_raiser_smooth)
            ###
            # Referencing from Table 1 in https://www.researchgate.net/publication/5584672_The_Automaticity_of_Emotion_Recognition
            # Taking the intersection of AUs under the "Female" and "Male" columns in that table
            ###############################
            # AU01 = Inner Brow Raiser    #
            # AU02 = Outer Brow Raiser    #
            # AU04 = Brow Lowerer         #
            # AU05 = Upper Lid Raiser     #
            # AU06 = Cheek Raiser         #
            # AU07 = Lid Tightener        #
            # AU09 = Nose Wrinkler        #
            # AU10 = Upper Lip Raiser     #
            # AU12 = Lip Corner Puller    #
            # AU14 = Dimpler              #
            # AU15 = Lip Corner Depressor #
            # AU17 = Chin Raiser          #
            # AU20 = Lip Stretcher        #
            # AU23 = Lip Tightener        #
            # AU25 = Lips Part            #
            # AU26 = Jaw Drop             #
            # AU28 = Lip Suck             #
            # AU45 = Blink                #
            ###############################
            # Anger: AU4 + AU7 + AU17 + AU23
            anger = max(aus['BrowLower'], aus['LidTighten'], aus['ChinRaise'], aus['LipFunnel']) if aus['BrowLower'] and aus['LidTighten'] and aus['ChinRaise'] and aus['LipFunnel'] else 0
            self.anger.update(anger, self.config.mirror_anger_smooth)
            # Disgust: AU4 + AU7 + AU9 + AU16 (not available) + AU19 (not available) + AU25 + AU26
            disgust = max(aus['BrowLower'], aus['LidTighten'], aus['NoseWrinkle'], aus['LipsApart'], aus['Jaw']) if aus['BrowLower'] and aus['LidTighten'] and aus['NoseWrinkle'] and aus['LipsApart'] and aus['Jaw'] else 0
            self.disgust.update(disgust, self.config.mirror_disgust_smooth)
            # Fear: AU1 + AU2 + AU4 + AU5 + AU25 + AU58 (not available)
            fear = max(aus['InnerBrowRaise'], aus['OuterBrowRaise'], aus['BrowLower'], aus['UpperLidRaise'], aus['LipsApart']) if aus['InnerBrowRaise'] and aus['OuterBrowRaise'] and aus['BrowLower'] and aus['UpperLidRaise'] and aus['LipsApart'] else 0
            self.fear.update(fear, self.config.mirror_fear_smooth)
            # Happiness: AU6 + AU7 + AU12 + AU25
            happiness = max(aus['CheeckRaise'], aus['LidTighten'], aus['Smile'], aus['LipsApart']) if aus['CheeckRaise'] and aus['LidTighten'] and aus['Smile'] and aus['LipsApart'] else 0
            self.happiness.update(happiness, self.config.mirror_happiness_smooth)
            # Sadness: AU1 + AU4 + AU15
            sadness = max(aus['InnerBrowRaise'], aus['BrowLower'], aus['Frown']) if aus['InnerBrowRaise'] and aus['BrowLower'] and aus['Frown'] else 0
            self.sadness.update(sadness, self.config.mirror_sadness_smooth)
            # Surprise: AU1 + AU2 + AU5 + AU25, AU38 (not available)
            surprise = max(aus['InnerBrowRaise'], aus['OuterBrowRaise'], aus['UpperLidRaise'], aus['LipsApart']) if aus['InnerBrowRaise'] and aus['OuterBrowRaise'] and aus['UpperLidRaise'] and aus['LipsApart'] else 0
            self.surprise.update(surprise, self.config.mirror_surprise_smooth)
        else:
            self.lip_corner_puller.update(0, self.config.mirror_lip_corner_puller_smooth)
            self.brows.update(0, self.config.mirror_brows_smooth)
            self.jaw_drop.update(0, self.config.mirror_jaw_drop_smooth)
            self.lip_corner_depressor.update(0, self.config.mirror_lip_corner_depressor_smooth)
            self.upper_lip_raiser.update(0, self.config.mirror_upper_lip_raiser_smooth)
            self.lip_tightener.update(0, self.config.mirror_lip_tightener_smooth)
            self.lip_stretcher.update(0, self.config.mirror_lip_stretcher_smooth)
            self.lips_part.update(0, self.config.mirror_lips_part_smooth)
            self.chin_raiser.update(0, self.config.mirror_chin_raiser_smooth)
            self.upper_lid_raiser.update(0, self.config.mirror_upper_lid_raiser_smooth)
            self.lid_tightener.update(0, self.config.mirror_lid_tightener_smooth)
            self.cheek_raiser.update(0, self.config.mirror_cheek_raiser_smooth)
            self.anger.update(0, self.config.mirror_anger_smooth)
            self.disgust.update(0, self.config.mirror_disgust_smooth)
            self.fear.update(0, self.config.mirror_fear_smooth)
            self.happiness.update(0, self.config.mirror_happiness_smooth)
            self.sadness.update(0, self.config.mirror_sadness_smooth)
            self.surprise.update(0, self.config.mirror_surprise_smooth)

    def reset(self):
        self.anger.reset()
        self.disgust.reset()
        self.fear.reset()
        self.happiness.reset()
        self.sadness.reset()
        self.surprise.reset()

    def configs_cb(self, cfg, level=None):
        if self.config is None:
            self.config = cfg
        self.config = cfg
        self.anger_exp.name = cfg.anger_exp
        self.disgust_exp.name = cfg.disgust_exp
        self.fear_exp.name = cfg.fear_exp
        self.happiness_exp.name = cfg.happiness_exp
        self.sadness_exp.name = cfg.sadness_exp
        self.surprise_exp.name = cfg.surprise_exp
        self.blink_anim.name = cfg.blink_anim
        return cfg

    def publish_mirroring(self):
        if not self.config.enabled:
            return
        # Blink
        if self.config.mirror_blink > 0 and len(self.blinks):
            while len(self.blinks):
                # blink happened before requested time
                if self.blinks[0] < time.time() - self.config.mirror_blink_delay:
                    blink = self.blinks.pop(0)
                    if blink - (time.time() - self.config.mirror_blink_delay) < 0.3:
                        self.animation_pub.publish(self.blink_anim)
                else:
                    break
        t = TargetPosture()
        # Brows
        if self.config.mirror_brows > 0:
            b = self.brows.get(time.time() - self.config.mirror_brows_delay)
            if b and abs(b) > self.config.mirror_brows_threshold:
                t.names = ['BrowOuterLeft', 'BrowInnerLeft','BrowCenter','BrowInnerRight','BrowOuterRight']
                t.values = [(b / 5) * self.config.mirror_brows] * 5
        # Jaw
        if self.config.mirror_jaw_drop > 0:
            j = self.jaw_drop.get(time.time() - self.config.mirror_jaw_drop_delay)
            if j and abs(j) > self.config.mirror_jaw_drop_threshold:
                t.names += ['Jaw']
                t.values += [(j / 5) * self.config.mirror_jaw_drop]
        # Head
        if self.config.mirror_head_roll > 0:
            r = self.roll.get(time.time() - self.config.mirror_head_roll_delay)
            if r and abs(r) > self.config.mirror_head_roll_threshold:
                reverse = 1 if self.config.mirror_head_roll_reverse else -1
                t.names += ['HeadRoll']
                t.values += [reverse * r * self.config.mirror_head_roll]
        if self.config.mirror_head_pitch > 0:
            r = self.pitch.get(time.time() - self.config.mirror_head_pitch_delay)
            if r and abs(r) > self.config.mirror_head_pitch_threshold:
                t.names += ['HeadPitch']
                t.values += [r * self.config.mirror_head_pitch]
        if self.config.mirror_head_yaw > 0:
            r = self.yaw.get(time.time() - self.config.mirror_head_yaw_delay)
            if r and abs(r) > self.config.mirror_head_yaw_threshold:
                t.names += ['HeadYaw']
                t.values += [r * self.config.mirror_head_yaw]
        # Lip
        stretch_val = 0
        low_lip_cent_val = 0
        if self.config.mirror_lip_corner_depressor > 0:
            l = self.lip_corner_depressor.get(time.time() - self.config.mirror_lip_corner_depressor_delay)
            if l and abs(l) > self.config.mirror_lip_corner_depressor_threshold:
                t.names += ['FrownLeft', 'FrownRight']
                t.values += [(l / 5) * self.config.mirror_lip_corner_depressor] * 2
        if self.config.mirror_lip_corner_puller > 0:
            l = self.lip_corner_puller.get(time.time() - self.config.mirror_lip_corner_puller_delay)
            if l and abs(l) > self.config.mirror_lip_corner_puller_threshold:
                t.names += ['SmileLeft', 'SmileRight']
                t.values += [(l / 5) * self.config.mirror_lip_corner_puller] * 2
        if self.config.mirror_upper_lip_raiser > 0:
            l = self.upper_lip_raiser.get(time.time() - self.config.mirror_upper_lip_raiser_delay)
            if l and abs(l) > self.config.mirror_upper_lip_raiser_threshold:
                t.names += ['UpperLipLeft', 'UpperLipCenter', 'UpperLipRight']
                t.values += [(l / 5) * self.config.mirror_upper_lip_raiser] * 3
        if self.config.mirror_lip_stretcher > 0:
            l = self.lip_stretcher.get(time.time() - self.config.mirror_lip_stretcher_delay)
            if l and abs(l) > self.config.mirror_lip_stretcher_threshold:
                stretch_val = (l / 5) * self.config.mirror_lip_stretcher
        if self.config.mirror_lip_tightener > 0:
            l = self.lip_tightener.get(time.time() - self.config.mirror_lip_tightener_delay)
            if l and abs(l) > self.config.mirror_lip_tightener_threshold:
                tighten_val = (l / 5) * self.config.mirror_lip_tightener
                if stretch_val:
                    stretch_val = stretch_val - tighten_val
                else:
                    stretch_val = -1 * tighten_val
        if stretch_val:
            t.names += ['LipsStretchLeft', 'LipsStretchRight']
            t.values += [stretch_val] * 2
        if self.config.mirror_lips_part > 0:
            l = self.lips_part.get(time.time() - self.config.mirror_lips_part_delay)
            if l and abs(l) > self.config.mirror_lips_part_threshold:
                low_lip_cent_val = (l / 5) * self.config.mirror_lips_part
                # Belows are smaller just to make it look more like a U-shape
                t.names += ['LowerLipLeft', 'LowerLipRight']
                t.values += [(l / 5) * self.config.mirror_lips_part * 0.2] * 2
        # Chin
        if self.config.mirror_chin_raiser > 0:
            c = self.chin_raiser.get(time.time() - self.config.mirror_chin_raiser_delay)
            if c and abs(c) > self.config.mirror_chin_raiser_threshold:
                chin_val = (c / 5) * self.config.mirror_chin_raiser
                if low_lip_cent_val:
                    low_lip_cent_val = low_lip_cent_val - chin_val
                else:
                    # Negative is upward
                    low_lip_cent_val = -1 * chin_val
        if low_lip_cent_val:
            t.names += ['LowerLipCenter']
            t.values += [low_lip_cent_val]
        # Lid
        if self.config.mirror_upper_lid_raiser > 0:
            l = self.upper_lid_raiser.get(time.time() - self.config.mirror_upper_lid_raiser_delay)
            if l and abs(l) > self.config.mirror_upper_lid_raiser_threshold:
                t.names += ['UpperLidLeft', 'UpperLidRight']
                t.values += [(l / 5) * self.config.mirror_upper_lid_raiser] * 2
        if self.config.mirror_lid_tightener > 0:
            l = self.lid_tightener.get(time.time() - self.config.mirror_lid_tightener_delay)
            if l and abs(l) > self.config.mirror_lid_tightener_threshold:
                t.names += ['UpperLidLeft', 'UpperLidRight', 'LowerLidLeft', 'LowerLidRight']
                t.values += [-1 * (l / 5) * self.config.mirror_lid_tightener] * 2
        # Cheek
        if self.config.mirror_cheek_raiser > 0:
            c = self.cheek_raiser.get(time.time() - self.config.mirror_cheek_raiser_delay)
            if c and abs(c) > self.config.mirror_cheek_raiser_threshold:
                t.names += ['CheekSquintLeft', 'CheekSquintRight']
                t.values += [(c / 5) * self.config.mirror_cheek_raiser] * 2
        # Emotions
        if self.config.mirror_anger > 0:
            a = self.anger.get(time.time() - self.config.mirror_anger_delay)
            if a and abs(a) > self.config.mirror_anger_threshold and (not self.expressing or time.time() - self.expressing >= self.expression_interval):
                self.anger_exp.magnitude = (a / 5) * self.config.mirror_anger
                self.expression_pub.publish(self.anger_exp)
                self.expression_magnitude = self.anger_exp.magnitude
                self.expressing = time.time()
                logger.info('Mirroring anger: {}'.format(round(self.anger_exp.magnitude, 3)))
        if self.config.mirror_disgust > 0:
            d = self.disgust.get(time.time() - self.config.mirror_disgust_delay)
            if d and abs(d) > self.config.mirror_disgust_threshold and (not self.expressing or time.time() - self.expressing >= self.expression_interval):
                self.disgust_exp.magnitude = (d / 5) * self.config.mirror_disgust
                self.expression_pub.publish(self.disgust_exp)
                self.expression_magnitude = self.disgust_exp.magnitude
                self.expressing = time.time()
                logger.info('Mirroring disgust: {}'.format(round(self.disgust_exp.magnitude, 3)))
        if self.config.mirror_fear > 0:
            f = self.fear.get(time.time() - self.config.mirror_fear_delay)
            if f and abs(f) > self.config.mirror_fear_threshold and (not self.expressing or time.time() - self.expressing >= self.expression_interval):
                self.fear_exp.magnitude = (f / 5) * self.config.mirror_fear
                self.expression_pub.publish(self.fear_exp)
                self.expression_magnitude = self.fear_exp.magnitude
                self.expressing = time.time()
                logger.info('Mirroring fear: {}'.format(round(self.fear_exp.magnitude, 3)))
        if self.config.mirror_happiness > 0:
            h = self.happiness.get(time.time() - self.config.mirror_happiness_delay)
            if h and abs(h) > self.config.mirror_happiness_threshold and (not self.expressing or time.time() - self.expressing >= self.expression_interval):
                self.happiness_exp.magnitude = (h / 5) * self.config.mirror_happiness
                self.expression_pub.publish(self.happiness_exp)
                self.expression_magnitude = self.happiness_exp.magnitude
                self.expressing = time.time()
                logger.info('Mirroring happiness: {}'.format(round(self.happiness_exp.magnitude, 3)))
        if self.config.mirror_sadness > 0:
            s = self.sadness.get(time.time() - self.config.mirror_sadness_delay)
            if s and abs(s) > self.config.mirror_sadness_threshold and (not self.expressing or time.time() - self.expressing >= self.expression_interval):
                self.sadness_exp.magnitude = (s / 5) * self.config.mirror_sadness
                self.expression_pub.publish(self.sadness_exp)
                self.expression_magnitude = self.sadness_exp.magnitude
                self.expressing = time.time()
                logger.info('Mirroring sadness: {}'.format(round(self.sadness_exp.magnitude, 3)))
        if self.config.mirror_surprise > 0:
            s = self.surprise.get(time.time() - self.config.mirror_surprise_delay)
            if s and abs(s) > self.config.mirror_surprise_threshold and (not self.expressing or time.time() - self.expressing >= self.expression_interval):
                self.surprise_exp.magnitude = (s / 5) * self.config.mirror_surprise
                self.expression_pub.publish(self.surprise_exp)
                self.expression_magnitude = self.surprise_exp.magnitude
                self.expressing = time.time()
                logger.info('Mirroring surprise: {}'.format(round(self.surprise_exp.magnitude, 2)))
        # If no emotion is being expressed for a certain period of time, reset
        if time.time() - self.expressing > self.expression_interval and self.expression_magnitude > 0:
            self.expression_pub.publish(self.neutral_exp)
            self.expression_magnitude = 0
            self.expressing = 0
        if len(t.values) > 0:
            self.external_pub.publish(t)


class SoundLocalization(object):

    def __init__(self, speaker_angle_callback):
        self.last_angle = 0
        self.last_angle_time = 0
        self.someone_speaking = False
        self.speaker_angle = None
        self.last_vad_time = 0
        self.last_angle_time = 0
        self.before_vad_timeout = 1.0
        self.after_vad_timeout = 0.0
        self.speaker_angle_cb = speaker_angle_callback
        rospy.Subscriber("/hr/perception/sound_direction", Int32, self.handle_angle)
        rospy.Subscriber("/hr/perception/hear/event", String, self.handle_vad)


    def handle_angle(self, msg):
        a = msg.data
        # disregard directions that are oputside visible range, and cant be reliably used
        if abs(a) > 90:
            return
        # Necessary
        self.last_angle_time = time.time()
        self.last_angle = a
        if self.someone_speaking or self.last_vad_time +self.after_vad_timeout > time.time():
            # Update speaker angle even after timeout
            self.speaker_angle = a
            try:
                self.speaker_angle_cb(self.speaker_angle)
            except:
                pass
        else:
            self.speaker_angle = None

    def handle_vad(self, msg):
        self.vad_detected(msg.data=="speechstart")

    def vad_detected(self, vad):
        # vad only alternates, so no need to check if previous was true/false
        if vad and self.last_angle_time + self.before_vad_timeout > time.time():
            self.speaker_angle = self.last_angle
            try:
                self.speaker_angle_cb(self.speaker_angle)
            except Exception as e:
                print(e)
                pass
        # regardless if true or false , if VAD message is alternating
        self.last_vad_time = time.time()
        self.someone_speaking = vad


    @staticmethod
    def angle(a):
        # Return 180 degreesh shifted angle
        return 180 - a if a > 0 else -180-a


class Person:
    # class contains all parameters to evauate tracked person need for attention
    def __init__(self, id, msg = None):
        # id
        self.id = id
        #last seen
        self.last_seen = time.time()
        self.first_seen = time.time()
        self.active = False
        # Time person become the main attention
        self.start_active = 0
        # Time person was last interacting (talking, maybe vawing, or manually triggered)
        self.keep_active = 0
        self.last_active = time.time()
        self.head_yaw = 0
        self.body_angle = 0
        self.distance = 0
        self.requested = False
        self.is_speaking = False
        self.rank = 0
        if msg is not None:
            self.update(msg)

    def start_attention(self):
        self.start_active = time.time()
        self.active = True

    def stop_attention(self):
        self.active = False
        self.last_active = time.time()

    def update(self, msg):
        self.last_seen = time.time()
        self.msg = msg
        self.body_angle = math.degrees(math.atan(msg.body.location.y/max(0.1,msg.body.location.x)))
        self.distance = (msg.body.location.y**2 + msg.body.location.x**2)**0.5

    def check_id(self, ids):
        ids = ids.split(",")
        for i in ids:
            i = i.strip()
            if(i):
                if i == self.id or i == self.msg.face.id:
                    return True
        return False


class PeopleAttention:
    """
        This is the model that based on parameters and perception data decides the person to track, and person to gaze.
        Currently its only tracking enabled, however gaze will be added.
    """
    def __init__(self):
        # Default ranking values
        self.cfg = {
            'keep_attention_min': 3, # try keep attention
            'keep_attention_cooling': 0.08, # exponential decreese of attention
            'increase_need_for_attention': 0.08, # exponential increase in need of attention
            'non_active_max_attention': 0.8,
            'min_speaker_attention': 0.88, # 1 - would make it return 1
            'switch_threshold': 0.01, # switch only if new face difference is higher. Could be used if in some cases there is a need to prevent robot from switching faces
            'keep_people_for': 2,
            'look_at_face': "",
            'look_at_time': 0,
            'look_at_inverse': False,
            'focus_during_tts': True,
        }
        self.tts_active = False
        self.last_spoken = None
        self.look_at_started = 0
        self.last_spoken_time = 0
        self.people = {}
        self.people_lock = threading.RLock()
        rospy.Subscriber('/hr/control/speech/event', String, self.tts_cb)

    def tts_cb(self, msg):
        if msg.data == 'start':
            self.tts_active = True
        elif msg.data == 'stop':
            self.tts_active = False


    def update_cfg(self, cfg):
        for k in self.cfg.keys():
            try:
                v = getattr(cfg, k)
                self.cfg[k] = v
            except:
                pass
        if cfg.look_at_start:
            self.look_at_started = time.time()


    def time_attention(self,person):
        """ Returns value 0 - 1 based on time and need for attention """
        if person.active:
            # during TTS person needs to be considered as currently speaking to robot, and as last person spoken to robot
            if self.tts_active and self.cfg['focus_during_tts']:
                person.keep_active = time.time()
                self.last_spoken = person.id
                self.last_spoken_time = person.keep_active
            # Keep active is reset whenever some peson interacts with robot, in that case robot will try to keep contact for longer
            interact_time = max(person.start_active or 0, person.keep_active or 0)
            # Active person need for attention lowers over time
            return (1-self.cfg['keep_attention_cooling'])**max(1, 1+time.time() - interact_time - self.cfg['keep_attention_min'])
        else:
            # currently inactive people need for more attention
            return 1-(1-self.cfg['increase_need_for_attention'])**max(1+time.time()-person.last_active,1)

    def rank(self, person):
        # Handle last spoken person
        # IF on last VAD event person spoke with robot it will keep attention to that person
        if person.id == self.last_spoken:
            person.keep_active = self.last_spoken_time
        else:
            person.keep_active = None

        rank = self.time_attention(person)
        # Only apply once so it only effects switching if necessary at that moment
        if person.is_speaking:
            print("DETECTED SPEAKING PERSON")
            rank = max(self.cfg['min_speaker_attention'], rank)
            person.is_speaking = False
        # Look at command started
        if self.look_at_started + self.cfg['look_at_time'] > time.time():
            if person.check_id(self.cfg['look_at_face']):
                if self.cfg['look_at_inverse']:
                    rank = rank*0.1
                else:
                    rank = 1.0
        person.rank = rank
        return rank

    def look_at_cmd(self, msg):
        if msg.x == 1.0:
            x = (1 -msg.y) * 320
            y = (0.5 -msg.z) * 480
            person = None
            # Distance in pixels to the face
            d = 200
            with self.people_lock:
                for p in self.people.values():
                    roi = p.msg.body.bounding_box
                    roi_x = roi.x_offset + roi.width/2.0
                    roi_y = roi.y_offset + roi.height/2.0
                    # Simple distance in image
                    dis = ((roi_x-x)**2 + (roi_y-y)**2)**0.5
                    if dis < d:
                        d = dis
                        person = p
                if person is not None:
                    self.cfg['look_at_face'] = person.id
                    self.cfg['look_at_time'] = 15
                    self.cfg['look_at_inverse'] = False
                    self.look_at_started = time.time()


    def find_person(self):
        # Main model that gives the best person to focus on
        self.remove_inactive()
        if len(self.people) < 1:
            return None
        max_rank = -100
        high_ranked_person = None
        active_rank = -100
        active_person = None
        #print()
        for p in self.people.values():
            rank = self.rank(p)
            #print('{}\t{}'.format(p.id, rank))
            if p.active:
                active_rank = rank
                active_person = p
            if rank > max_rank:
                high_ranked_person = p
                max_rank = rank
        # Check threshold, and switch if needed
        switch = False
        if active_rank + self.cfg['switch_threshold'] < max_rank:
            switch = True
            if active_person is not None:
                active_person.stop_attention()

        if not high_ranked_person.active and switch:
            high_ranked_person.start_attention()

        # for p in self.people.values():
        #     print("{} active {}: {}".format(p.id[-3:], p.rank, 'y'if p.active else 'n'))
        # print("-------")
        return high_ranked_person if switch else active_person

    def remove_inactive(self):
        with self.people_lock:
            for i in list(self.people.keys())[:]:
                if self.people[i].last_seen + self.cfg['keep_people_for'] < time.time():
                    del self.people[i]


    def update_people(self, msg):
        with self.people_lock:
            for p in msg.people:
                if p.id in list(self.people.keys())[:]:
                    self.people[p.id].update(p)
                else:
                    self.people[p.id] = Person(p.id, p)

    def update_speaker(self, angle):
        with self.people_lock:
            max_angle = 30
            speaker = None
            for p in self.people.values():
                # print("{} angle diff : {}".format(p.id[-3:], abs(p.body_angle-angle)))
                if abs(p.body_angle-angle) < max_angle:
                    max_angle = abs(p.body_angle-angle)
                    speaker = p
            if speaker is not None:
                #print(speaker.id[-3:])
                speaker.is_speaking =True
                # Only single person could be last spoken
                self.last_spoken = speaker.id
                self.last_spoken_time = time.time()

class Filters:

    def __init__(self):
        self.samples = 0
        self.tracks = {}

    def ema_update(self, name, v):
        self.tracks[name]['value'] = v*(self.tracks[name]['smooth']/(1+self.tracks[name]['window'])) + self.tracks[name]['value'] * (1- self.tracks[name]['smooth']/(1+self.tracks[name]['window']))
        self.tracks[name]['samples'] += 1

    def ema_reset(self, name):
        self.tracks[name]['value'] = self.tracks[name]['start']
        self.tracks[name]['samples'] = 0

    def ema(self, name, start=0, smooth=2, window=10):
        self.tracks[name] = {
            'start': start,
            'smooth': float(smooth),
            'window': float(window),
            'value': start,
            'update': lambda v: self.ema_update(name, v),
            'reset': lambda: self.ema_reset(name),
            'samples': 0,
        }

    def update(self, name, value):
        self.samples += 1
        self.tracks[name]['update'](value)
        return self.get(name)

    def reset(self):
        # Resets all if name is not given
        self.samples = 0
        for t in self.tracks:
            t['reset']()

    def get(self, name):
        return self.tracks[name]['value']

class IdleAttention:
    
    def __init__(self) -> None:
        self.cfg = {
            # Bored attention controller (when faces are not visible. Camera is off)
            "looking_around": True,
            "interval_min": 10,
            "interval_max": 10,
            "head_speed": 1.0,
            "range_x": 1.0,
            "range_y": 1.0,
            "change_x": 1.0,
            "change_y": 1.0, 
            "eyes_x": 0.2,
            "eyes_y": 0.0,
        }
        # left/right, top-bottom
        self.last_target = [0, 0]
        self.time_left = 5
        pass
    
    def update_cfg(self, cfg):
        for k in self.cfg.keys():
            try:
                v = getattr(cfg, f"bored_{k}")
                self.cfg[k] = v
            except:
                pass

    def generate_random_target(self):
        x = random.uniform(-self.cfg['range_x'], self.cfg['range_x'])
        y = random.uniform(-self.cfg['range_y'], self.cfg['range_y'])
        return [x, y]
    
    def generate_new_target(self):
        direction = self.generate_random_target()
        # use 0.4 as default max change of target and adjust based on scale:
        new_target = self.last_target
        new_target[0] = sorted([new_target[0]-self.cfg['change_x'], new_target[0]+self.cfg['change_x'], direction[0]])[1]
        new_target[1] = sorted([new_target[1]-self.cfg['change_y'], new_target[1]+self.cfg['change_y'], direction[1]])[1]
        return new_target

    def make_pos(self, target):
        return Point(x=1.0,y=target[0], z = target[1])

    def split_to_head_eyes_pos(self, target):
        # make certain parts 
        head_x = 1 - random.random() * self.cfg['eyes_x']
        head_y = 1 - random.random() * self.cfg['eyes_y']     
        head = [target[0]*head_x, target[1]*head_y]
        return [self.make_pos(head), self.make_pos(target)]
    
    def reset(self):
        self.last_target = [0, 0]
        self.time_left = random.uniform(self.cfg['interval_min'], self.cfg['interval_max'])

    def update(self, dt, active):
        if not self.cfg['looking_around']:
            return None
        if not active:
            self.reset()
            return None
        if self.time_left > dt:
            self.time_left -= dt
            return None
        # Time to make new target
        self.time_left = random.uniform(self.cfg['interval_min'], self.cfg['interval_max'])
        target = self.generate_new_target()
        # Returns position for heads and eyes
        return self.split_to_head_eyes_pos(target)


class Attention:

    def __init__(self):

        # create lock
        self.lock = threading.RLock()
        # Look at people
        self.people_model = PeopleAttention() # People ranking model
        self.sound_localization = SoundLocalization(self.people_model.update_speaker)
        self.mirroring = Mirroring()
        self.current_person = None
        self.is_idle = True
        self.configs_init = False
        self.cfg = None
        self.idle_attention = IdleAttention()
        self.config_server = Server(AttentionConfig, self.configs_cb, namespace='/hr/behavior/attention')
        while not self.configs_init:
            time.sleep(0.1)
        self.timers = {}

        self.tf_listener = tf.TransformListener(False, rospy.Duration.from_sec(1))
        rospy.Subscriber('/hr/perception/people', People, self.people_cb)
        rospy.Subscriber('/hr/animation/set_face_target', Target, self.people_model.look_at_cmd)
        self.head_focus_pub = rospy.Publisher('/hr/animation/set_face_target', Target, queue_size=1)
        self.gaze_focus_pub = rospy.Publisher('/hr/animation/set_gaze_target', Target, queue_size=1)
        self.head_tilt_pub = rospy.Publisher('/hr/animation/set_head_tilt', Float32 , queue_size=1)
        self.animationmode_pub = rospy.Publisher('/hr/animation/set_animation_mode', UInt8, queue_size=1)
        # Topic to publish current person
        self.current_target_pub = rospy.Publisher('/hr/behavior/current_person', String, queue_size=1, latch=True)
        self.current_face_pub = rospy.Publisher('/hr/behavior/current_face', String, queue_size=1, latch=True)
        self.last_face = None
        self.last_person = None


    def handle_target(self, msg):
        # TODO move to attention class
        return

    def publish_current_person(self):
        if self.current_person is not None:
            if self.current_person.id != self.last_person:
                self.last_person = self.current_person.id
                self.current_target_pub.publish(self.current_person.id)
                self.mirroring.reset()
            self.mirroring.update_faces(self.current_person.msg)
            try:
                face = self.current_person.msg.face.id or 'unknown'
            except:
                face = 'unknown'
            if face != self.last_face:
                self.last_face = face
                self.current_face_pub.publish(face)
        else:
            if self.last_person:
                self.last_person = ""
                self.current_target_pub.publish("")
            if self.last_face:
                self.last_face = ""
                self.current_face_pub.publish("")



    def start_mimic(self):
        self.mimic_active=True


    def reset_head_pose(self):
        self.head_tilt_pub.publish(0)

    def idle(self):
        # Return robot to idle. Only once
        self.UpdateGaze(Point(x=1,y=0,z=0),rospy.Time(0), 'blender')

    def configs_cb(self, config, level):
        with self.lock:
            self.configs_init = True
            self.people_model.update_cfg(config)
            self.idle_attention.update_cfg(config)
            if config.look_at_start:
                config.look_at_start = False
            self.cfg = config
        return config

    def get_blender_pos(self, pos, ts, frame_id):
        if frame_id == 'blender':
            return pos
        else:
            ps = PointStamped()
            ps.header.seq = 0
            ps.header.stamp = ts
            ps.header.frame_id = frame_id
            ps.point.x = pos.x
            ps.point.y = pos.y
            ps.point.z = pos.z
            pst = self.tf_listener.transformPoint("blender", ps)
            return pst.point


    @staticmethod
    def pos_to_target(pos, speed):
        msg = Target()
        msg.x = max(0.3, pos.x)
        msg.y = pos.y if not math.isnan(pos.y) else 0
        msg.z = pos.z if not math.isnan(pos.z) else 0
        msg.z = max(-0.3, min(0.3, msg.z))
        if pos.x < 0.3:
            msg.z = 0
            msg.y = 0
        msg.speed = speed
        return msg

    def SetGazeFocus(self, pos, speed, ts, frame_id='robot'):
        try:
            pos = self.get_blender_pos(pos, ts, frame_id)
            msg = self.pos_to_target(pos, speed)
            self.gaze_focus_pub.publish(msg)
        except Exception as e:
            logger.warning("Gaze focus exception: {}".format(e))

    def SetHeadFocus(self, pos, speed, ts, frame_id='robot', proportional=True):
        try:
            pos = self.get_blender_pos(pos, ts, frame_id)
            msg = self.pos_to_target(pos, speed)
            if proportional:
                msg.y = msg.y * self.cfg.head_yaw_movement
                msg.z = msg.z * self.cfg.head_pitch_movement

            self.head_focus_pub.publish(msg)
        except Exception as e:
            logger.warning("Head focus exception: {}".format(e))

    def UpdateGaze(self, pos, ts, frame_id="realsense"):
        self.SetGazeFocus(pos, 5.0, ts, frame_id)
        self.SetHeadFocus(pos, self.cfg.head_speed, ts, frame_id)

    def update_current_target(self):
        person = self.people_model.find_person()
        if person is None:
            self.current_person = None
            # if not self.is_idle:
            #     self.idle()
            return
        changed = False
        if self.current_person is None or person.id != self.current_person.id:
            changed = True
        self.current_person = person
        self.is_idle = False

    # def SelectNextRegion(self):
    #     # Check if setperformance has set regions
    #     regions = rospy.get_param("/hr/control/performance_regions", {})
    #     if len(regions) == 0:
    #         regions = rospy.get_param("/hr/control/regions", {})
    #     point = AttentionRegion.get_point_from_regions(regions, REGIONS[self.attention_region])
    #     return Point(x=point['x'], y=point['y'], z=point['z'])

    def update(self, dt):
        if not self.cfg.enable_flag:
            # Disabled do nothing
            return
        # Process all timers
        for timer in self.timers.values():
            timer.update(dt)
        # Update current_person
        with self.lock:
            self.update_current_target()
        # Update Head_ position
        if self.current_person is not None:
            self.update_face(self.current_person)
            self.UpdateGaze(self.current_person.msg.body.location, rospy.Time(0), 'realsense')
        # Idle attention is active if no person is visible
        idle_targets = self.idle_attention.update(dt, self.current_person is None)
        if idle_targets is not None:
            self.SetHeadFocus(idle_targets[0], self.cfg.bored_head_speed, rospy.Time(0), 'blender', False)
            self.SetGazeFocus(idle_targets[1], 5.0, rospy.Time(0), 'blender')


    def update_face(self, person):
        return
        if self.cfg.mimic_enabled:
            face = self.current_person.face
            q = face.head_pose.orientation
            e = euler_from_quaternion([q.x,q.y,q.z,q.w])
            r = math.pi + e[2] if e[2] < 0 else e[2] - math.pi
            # Ignore invalid values
            if abs(r) > math.pi/4.0:
                r = 0
            # Smoothen roll
            roll = self.filters.update('head_roll', r)

            if self.mimic_active:
                # mimic Roll
                self.head_tilt_pub.publish(roll)
            # degrees = [math.degrees(a) for a in e]
            # r = 180 + degrees[2] if degrees[2] < 0 else degrees[2] - 180
            # p = 180 + degrees[0] if degrees[0] < 0 else degrees[0] - 180
            # y = degrees[1]
            # rr = self.filters.update('head_roll', r)
            # pp = self.filters.update('head_pitch', p)



    def change_person(self):
        pass

    def StartPauMode(self):

        mode = UInt8()
        mode.data = 148
        self.animationmode_pub.publish(mode)


    def StopPauMode(self):

        mode = UInt8()
        mode.data = 0
        self.animationmode_pub.publish(mode)


    def people_cb(self, msg):
        with self.lock:
            self.people_model.update_people(msg)

if __name__ == "__main__":
    rospy.init_node('attention')
    node = Attention()
    r = 50
    rate = rospy.Rate(r)
    while not rospy.is_shutdown():
        node.mirroring.publish_mirroring()
        node.update(1/float(r))
        node.publish_current_person()
        rate.sleep()

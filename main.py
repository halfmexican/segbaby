#!/usr/bin/env pybricks-micropython

# Segbaby EV3
# Copyright (C) 2025 Jose H
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor, TouchSensor
from pybricks.parameters import Port, Color
from pybricks.tools import wait, StopWatch

# Objects
ev3 = EV3Brick()
right_motor = Motor(Port.B)
left_motor = Motor(Port.D)
gyro_sensor = GyroSensor(Port.S2)
us_sensor = UltrasonicSensor(Port.S3)
touch_sensor = TouchSensor(Port.S1)

# Timers
loop_timer = StopWatch()
fall_timer = StopWatch()
turn_timer = StopWatch()
backup_timer = StopWatch()

#Constants 

#gyro
GYRO_CALIBRATION_LOOP_COUNT = 200
GYRO_OFFSET_FACTOR = 0.0005
LOOP_MS = 20
DT = LOOP_MS / 1000

FORWARD_SPEED = 275
BACKUP_DURATION_MS = 10000
BACKUP_SPEED = -50

OBSTACLE_THRESHOLD_MM = 400
TOO_CLOSE_THRESHOLD_MM = 200

TURN_DURATION_MS = 1600
TURN_STEERING = 90
RAMP_STEP = 1
BACKUP_RAMP_STEP = 2

def stop_motors():
    left_motor.stop()
    right_motor.stop()

def print_debug(msg):
    ev3.speaker.set_speech_options(language='es-la')
    print('[DEBUG‑ES]', msg)

def start_backup_sequence():
    print_debug('Pared golpeada - retrocediendo') # Wall hit - backing up
    backing_up = True
    backup_timer.reset()
    drive_speed = 0
    tgt_speed = BACKUP_SPEED
    tgt_steer = TURN_STEERING if turn_left_next else -TURN_STEERING
    return backing_up, drive_speed, tgt_speed, tgt_steer

def start_turn_sequence(turn_left: bool):
    print_debug('Iniciando giro') # Starting turn
    turning = True
    turn_timer.reset()
    tgt_speed = FORWARD_SPEED
    tgt_steer = TURN_STEERING if turn_left else -TURN_STEERING
    return turning, tgt_speed, tgt_steer

def main():
    while True:
        ev3.light.off()
        print_debug('Calibrando giroscopio')
        while True:
            g_min, g_max, g_sum, valid = 440, -440, 0, 0
            for _ in range(GYRO_CALIBRATION_LOOP_COUNT):
                g = gyro_sensor.speed()
                if g is None:
                    wait(5)
                    continue
                g_sum += g
                valid += 1
                g_min = min(g_min, g)
                g_max = max(g_max, g)
                wait(5)
            if valid and (g_max - g_min < 2):
                break

        gyro_offset = g_sum / valid
        ev3.light.on(Color.GREEN)
        print_debug('¡Listo para avanzar!') # Ready to move forward!
        wait(200)

        left_motor.reset_angle(0)
        right_motor.reset_angle(0)
        fall_timer.reset()

        tgt_speed, tgt_steer = FORWARD_SPEED, 0
        drive_speed, steering = 0, 0
        turning = False
        backing_up = False
        global turn_left_next
        turn_left_next = True

        body_angle = 0
        motor_sum = 0
        wheel_angle = 0
        motor_delta_buf = [0, 0, 0, 0]
        dist_mm = 0

        while True:
            loop_timer.reset()

            g = gyro_sensor.speed()
            if g is None:
                wait(1)
                continue
            
            dist_mm = us_sensor.distance()


            # Gyro offset logic
            gyro_offset = (1 - GYRO_OFFSET_FACTOR) * gyro_offset + GYRO_OFFSET_FACTOR * g
            body_rate = g - gyro_offset

            # Integrate body_angle
            body_angle += body_rate * DT

            # Wheel kinematics
            prev_sum = motor_sum
            motor_sum = left_motor.angle() + right_motor.angle()
            d_pos = motor_sum - prev_sum
            motor_delta_buf.insert(0, d_pos)
            motor_delta_buf.pop()
            wheel_angle += d_pos - drive_speed * DT
            wheel_rate = sum(motor_delta_buf) / 4 / DT

            # Controller
            angle_error = (body_angle)
            out = (
                -0.01 * drive_speed
                + 1.2  * body_rate
                + 28   * angle_error
                + 0.075* wheel_rate
                + 0.12 * wheel_angle
            )
            out = max(min(out, 100), -100)

            left_motor.dc(out - 0.1 * steering)
            right_motor.dc(out + 0.1 * steering)

            if abs(out) < 100:
                fall_timer.reset()
            elif fall_timer.time() > 1000:
                print_debug('He caído :(')
                break

            # Check states
            if backing_up:
                # Finished backing up?
                if backup_timer.time() >= BACKUP_DURATION_MS:
                    print_debug('Retroceso completado, iniciando giro') # Backup complete, starting turn

                    backing_up = False
                    (turning,
                     tgt_speed,
                     tgt_steer) = start_turn_sequence(turn_left_next)

                    # Flip turn_left_next for the next time around
                    turn_left_next = not turn_left_next
                else:
                    # Ramp speed while backing up
                    drive_speed += max(-BACKUP_RAMP_STEP,min(BACKUP_RAMP_STEP, BACKUP_SPEED - drive_speed))
            elif turning:
                if turn_timer.time() >= TURN_DURATION_MS:
                    print_debug('Giro completado, avanzando') # Turn complete, going forward
                    turning = False
                    turn_left_next = not turn_left_next
                    tgt_speed, tgt_steer = FORWARD_SPEED, 0
                else:
                    if touch_sensor.pressed():
                        backing_up, drive_speed, tgt_speed, tgt_steer = start_backup_sequence()
                    else:
                        dist_mm = us_sensor.distance() or 10000
                        if dist_mm < OBSTACLE_THRESHOLD_MM:
                            #print_debug('Obstáculo detectado - desviando') # Obstacle detected - deviating
                            turn_timer.reset()

            else:
                if touch_sensor.pressed() or dist_mm < TOO_CLOSE_THRESHOLD_MM:
                    backing_up, drive_speed, tgt_speed, tgt_steer = start_backup_sequence()
                elif dist_mm < OBSTACLE_THRESHOLD_MM:
                   # print_debug('Obstáculo detectado - desviando') # Obstacle detected - deviating
                    turning = True
                    turn_timer.reset()
                    tgt_speed = FORWARD_SPEED  
                    tgt_steer = TURN_STEERING if turn_left_next else -TURN_STEERING

            if not backing_up:
                drive_speed += max(-RAMP_STEP, min(RAMP_STEP, tgt_speed - drive_speed))
                steering += max(-RAMP_STEP, min(RAMP_STEP, tgt_steer - steering))
            else:
                # Adjust steering towards tgt_steer during backup
                steering += max(-RAMP_STEP, min(RAMP_STEP, tgt_steer - steering))

            wait(max(0, LOOP_MS - loop_timer.time()))

        stop_motors()
        ev3.light.on(Color.RED)
        wait(3000)

if __name__ == '__main__':
    try:
        ev3.speaker.say('Hola')
        main()
    except KeyboardInterrupt:
        stop_motors()
        print_debug('Programa terminado por el usuario') 
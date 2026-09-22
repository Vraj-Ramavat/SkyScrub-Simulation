#!/usr/bin/env python3

from pathlib import Path

repo = Path(__file__).resolve().parents[1]
sdf_path = repo / "models" / "skyscrub" / "model.sdf"

text = sdf_path.read_text()

sensor_name = 'front_facade_range'

if f'name="{sensor_name}"' in text:
    print("Front facade range sensor already exists.")
    raise SystemExit(0)

base_start = text.index('<link name="base_link">')
base_end = text.index('    </link>', base_start)

sensor = r'''
      <!-- SkyScrub V0.3 front facade standoff sensor -->
      <sensor name="front_facade_range" type="gpu_lidar">
        <!-- +X is forward. Sensor is slightly ahead of the body. -->
        <pose>0.22 0 0.05 0 0 0</pose>

        <topic>/skyscrub/front/range</topic>

        <always_on>true</always_on>
        <update_rate>20</update_rate>
        <visualize>true</visualize>

        <ray>
          <scan>
            <horizontal>
              <samples>5</samples>
              <resolution>1</resolution>

              <!-- Narrow approximately 2 degree field of view -->
              <min_angle>-0.0174533</min_angle>
              <max_angle>0.0174533</max_angle>
            </horizontal>

            <vertical>
              <samples>1</samples>
              <resolution>1</resolution>
              <min_angle>0</min_angle>
              <max_angle>0</max_angle>
            </vertical>
          </scan>

          <range>
            <min>0.20</min>
            <max>20.0</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
      </sensor>

'''

text = text[:base_end] + sensor + text[base_end:]

sdf_path.write_text(text)

print("Added V0.3 front facade range sensor.")
print(f"SDF: {sdf_path}")
print("Topic: /skyscrub/front/range")

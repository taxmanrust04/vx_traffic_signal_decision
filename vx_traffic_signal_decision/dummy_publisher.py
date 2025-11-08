#!/usr/bin/env python3

import paho.mqtt.publish as publish
import argparse
 
def force_traffic_light_red(signal_group=None):
    """
    Force traffic light(s) to RED via MQTT override
    Args:
        signal_group: None (all lights) or specific signal group ID (1-8)
    """
    # MQTT Configuration
    mqtt_host = "10.8.50.73"  # Raspberry Pi IP
    mqtt_port = 1883
    mqtt_topic = "modelcity/tls/override"
    mqtt_qos = 2

    # Build override pattern
    if signal_group is None:
        # Force ALL lights RED
        override_pattern = "33333333"         # Phase 3 = RED for all 8 signals
        message = "All traffic lights RED"
    else:
        # Force specific signal group RED, others GREEN
        if not (1 <= signal_group <= 8):
            print(f"Error: signal_group must be 1-8, got {signal_group}")
            return
        pattern_list = list("55555555")       # Default: all GREEN
        pattern_list[signal_group - 1] = "3"  # Set specific light to RED
        override_pattern = "".join(pattern_list)
        message = f"Signal Group {signal_group} RED"

    # Publish MQTT message
    publish.single(
            mqtt_topic,
            override_pattern,
            qos=mqtt_qos,
            hostname=mqtt_host,
            port=mqtt_port
        )
 
def main():
    parser = argparse.ArgumentParser(description='Force traffic light RED')
    parser.add_argument('--signal', type=int, default=None,
                      help='Signal group ID (1-8). If not specified, forces all lights RED')
    args = parser.parse_args()
    force_traffic_light_red(args.signal)
 
if __name__ == '__main__':
    main()


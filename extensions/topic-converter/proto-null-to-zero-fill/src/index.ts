import { ExtensionContext, Immutable } from "@foxglove/extension";

// Type definitions for our sensor data
interface SensorData {
  raw_value: number;
  value: number;
  sensor_status: number;
}

interface SensorsMessage {
  sensors: {
    [key: string]: SensorData | null;
  };
}

// Default sensor data when a ducer_out_N is null
const DEFAULT_SENSOR_DATA: SensorData = {
  raw_value: 0,
  value: 0,
  sensor_status: 0,
};

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerMessageConverter({
    type: "topic",
    inputTopics: ["GROUP_ID_PRIMSCATPActuatorSensorFrame"],
    outputTopic: "GROUP_ID_PRIMSCATPActuatorSensorFrame/show_nulls_as_zeros",
    outputSchemaName: "NormalizedActuatorSensorFrame",
    outputSchemaDescription: {
      sensors: {
        // Dynamic map: any "ducer_out_*" key maps to sensor data
        "[key: string]": {
          raw_value: "number", 
          value: "number", 
          sensor_status: "number"
        },
      },
    },
    create: () => {
      return (msgEvent) => {
        const msg = msgEvent.message as Immutable<SensorsMessage>;
        const normalizedSensors: { [key: string]: SensorData } = {};

        // Process each sensor field
        for (const [key, value] of Object.entries(msg.sensors)) {
          if (value === null || value === undefined) {
            // Replace null/undefined with default zeroed values
            normalizedSensors[key] = { ...DEFAULT_SENSOR_DATA };
          } else {
            // Keep existing non-null values
            normalizedSensors[key] = {
              raw_value: value.raw_value,
              value: value.value,
              sensor_status: value.sensor_status,
            };
          }
        }

        return {
          sensors: normalizedSensors,
        };
      };
    },
  });
}

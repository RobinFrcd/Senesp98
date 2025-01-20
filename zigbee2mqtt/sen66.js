const {
    temperature, 
    humidity, 
    pm25, 
    co2,
    numeric,
    deviceAddCustomCluster
} = require('zigbee-herdsman-converters/lib/modernExtend');
const {Zcl} = require('zigbee-herdsman');
const {logger} = require('zigbee-herdsman-converters/lib/logger');

const NS = 'zhc:sensirion';

const addCustomClusters = () => [
    deviceAddCustomCluster('sen66Pm1Meas', {
        ID: 0xFC01,
        attributes: {
            measuredValue: {ID: 0x0000, type: Zcl.DataType.SINGLE_PREC}
        },
        commands: {},
        commandsResponse: {},
    }),
    deviceAddCustomCluster('sen66Pm4Meas', {
        ID: 0xFC02,
        attributes: {
            measuredValue: {ID: 0x0000, type: Zcl.DataType.SINGLE_PREC}
        },
        commands: {},
        commandsResponse: {},
    }),
    deviceAddCustomCluster('sen66Pm10Meas', {
        ID: 0xFC03,
        attributes: {
            measuredValue: {ID: 0x0000, type: Zcl.DataType.SINGLE_PREC}
        },
        commands: {},
        commandsResponse: {},
    }),
    deviceAddCustomCluster('sen66NoxMeas', {
        ID: 0xFC04,
        attributes: {
            measuredValue: {ID: 0x0000, type: Zcl.DataType.SINGLE_PREC}
        },
        commands: {},
        commandsResponse: {},
    }),
    deviceAddCustomCluster('sen66VocMeas', {
        ID: 0xFC05,
        attributes: {
            measuredValue: {ID: 0x0000, type: Zcl.DataType.SINGLE_PREC}
        },
        commands: {},
        commandsResponse: {},
    }),
];

const definition = {
    zigbeeModel: ['SEN66'],
    model: 'SEN66',
    vendor: 'Sensirion',
    description: 'Custom multi-sensor device with VOC, PM1, PM4, PM10, and NOx measurements',
    extend: [
        temperature(), 
        humidity(), 
        pm25(), 
        co2(),
        ...addCustomClusters(),
        // Ranges according to https://sensirion.com/media/documents/FAFC548D/6731FFFA/Sensirion_Datasheet_SEN6x.pdf
        numeric({
            name: 'pm1',
            cluster: 'sen66Pm1Meas',
            attribute: 'measuredValue',
            unit: 'µg/m³',
            access: 'STATE_GET',
            reporting: {
                min: 0,
                max: 1000,
                change: 1
            },
            precision: 0,
            description: 'Measured PM1 value'
        }),
        numeric({
            name: 'pm4',
            cluster: 'sen66Pm4Meas',
            attribute: 'measuredValue',
            unit: 'µg/m³',
            access: 'STATE_GET',
            reporting: {
                min: 0,
                max: 1000,
                change: 1
            },
            precision: 0,
            description: 'Measured PM4 value'
        }),
        numeric({
            name: 'pm10',
            cluster: 'sen66Pm10Meas',
            attribute: 'measuredValue',
            unit: 'µg/m³',
            access: 'STATE_GET',
            reporting: {
                min: 0,
                max: 1000,
                change: 1
            },
            precision: 0,
            description: 'Measured PM10 value'
        }),
        numeric({
            name: 'nox',
            cluster: 'sen66NoxMeas',
            attribute: 'measuredValue',
            unit: 'index',
            access: 'STATE_GET',
            reporting: {
                min: 0,
                max: 500,
                change: 1
            },
            description: 'Measured NOx value'
        }),
        numeric({
            name: 'voc',
            cluster: 'sen66VocMeas',
            attribute: 'measuredValue',
            unit: 'index',
            access: 'STATE_GET',
            reporting: {
                min: 0,
                max: 500,
                change: 1
            },
            description: 'Measured VOC value'
        })
    ],
    configure: async (device, coordinatorEndpoint) => {
        const endpoint = device.getEndpoint(1);
        const clusters = [0xFC01, 0xFC02, 0xFC03, 0xFC04, 0xFC05];
        
        for (const cluster of clusters) {
            try {
                await endpoint.bind(cluster, coordinatorEndpoint);
                await endpoint.configureReporting(cluster, [{
                    attribute: 'measuredValue',
                    minimumReportInterval: 10,
                    maximumReportInterval: 3600,
                    reportableChange: 1
                }]);
                logger.info(`Configured cluster ${cluster} for device ${device.ieeeAddress}`, NS);
            } catch (error) {
                logger.warning(`Failed to configure cluster ${cluster}: ${error}`, NS);
            }
        }
    },
    meta: {
        configureKey: 1,
    },
};

module.exports = definition;
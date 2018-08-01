import { Matrix4 } from 'three'
import { Vector3 } from 'three'
import { Quaternion } from 'three'

import { message } from '../../shared/proto/messages'
import { Imat4 } from '../../shared/proto/messages'
import { Simulator } from '../simulator'
import { Message } from '../simulator'
import Sensors = message.input.Sensors

export const HIP_TO_FOOT = 0.2465

export class SensorDataSimulator implements Simulator {
  static of() {
    return new SensorDataSimulator()
  }

  simulate(time: number, index: number, numRobots: number): Message[] {
    const messageType = 'message.input.Sensors'

    // Simulate a walk
    const t = time * 5 + index

    const angle = index * (2 * Math.PI) / numRobots + time / 40
    const distance = Math.cos(time + 4 * index) * 0.3 + 1
    const x = distance * Math.cos(angle)
    const y = distance * Math.sin(angle)
    const heading = angle + Math.PI
    const Htw = toHtw(x, y, HIP_TO_FOOT, heading)

    const buffer = Sensors.encode({
      world: toProtoMat44(Htw),
      servo: [
        { presentPosition: 3 * Math.PI / 4 + 0.5 * Math.cos(t - Math.PI) },
        { presentPosition: 3 * Math.PI / 4 + 0.5 * Math.cos(t) },
        { presentPosition: -Math.PI / 8 },
        { presentPosition: Math.PI / 8 },
        { presentPosition: -3 * Math.PI / 4 },
        { presentPosition: -3 * Math.PI / 4 },
        { presentPosition: 0 },
        { presentPosition: 0 },
        { presentPosition: 0 },
        { presentPosition: 0 },
        { presentPosition: 0.5 * (Math.cos(t) - 1) },
        { presentPosition: 0.5 * (Math.cos(t - Math.PI) - 1) },
        { presentPosition: 0.5 * (-Math.cos(t) + 1) },
        { presentPosition: 0.5 * (-Math.cos(t - Math.PI) + 1) },
        { presentPosition: 0 },
        { presentPosition: 0 },
        { presentPosition: 0 },
        { presentPosition: 0 },
        { presentPosition: 0.1 * Math.cos(t) },
        { presentPosition: 0.1 * Math.cos(t / 3) + 0.4 },
      ],
      timestamp: {
        seconds: Math.floor(time),
        nanos: (time - Math.floor(time)) * 1e9,
      },
    }).finish()

    const message = { messageType, buffer }

    return [message]
  }
}

function toHtw(x: number, y: number, z: number, heading: number): Matrix4 {
  const translation = new Vector3(x, y, z)
  const rotation = new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), heading)
  const scale = new Vector3(1, 1, 1)
  return new Matrix4().getInverse(new Matrix4().compose(translation, rotation, scale))
}

function toProtoMat44(m: Matrix4): Imat4 {
  return {
    x: { x: m.elements[0], y: m.elements[1], z: m.elements[2], t: m.elements[3] },
    y: { x: m.elements[4], y: m.elements[5], z: m.elements[6], t: m.elements[7] },
    z: { x: m.elements[8], y: m.elements[9], z: m.elements[10], t: m.elements[11] },
    t: { x: m.elements[12], y: m.elements[13], z: m.elements[14], t: m.elements[15] },
  }
}
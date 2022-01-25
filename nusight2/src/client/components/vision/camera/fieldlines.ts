import { createTransformer } from 'mobx-utils'
import * as THREE from 'three'
import { Vector3 } from '../../../math/vector3'
import { Vector4 } from '../../../math/vector4'
import { group } from '../../three/builders'
import { Canvas } from '../../three/three'
import { LineProjection } from './line_projection'
import { CameraParams, FieldLine } from './model'



export class FieldLinesViewModel {
  private readonly model: FieldLine[]
  private readonly params: CameraParams
  private readonly lineProjection: LineProjection

  constructor(model: FieldLine[], params: CameraParams, lineProjection: LineProjection) {
    this.model = model
    this.params = params
    this.lineProjection = lineProjection
  }

  static of(model: FieldLine[], canvas: Canvas, params: CameraParams): FieldLinesViewModel {
    return new FieldLinesViewModel(model, params, LineProjection.of(canvas, params.lens))
  }

  readonly fieldlines = group(() => ({
    children: this.model.map(fieldline => this.fieldline(fieldline)),
  }))

  private fieldline = createTransformer((fieldline: FieldLine) => {
    // Transform the line so that it is in the perspective of the latest camera image.
    // Camera to world transform of the line when it was created
    const Hwc = new THREE.Matrix4().getInverse(fieldline.Hcw.toThree())
    // The two endpoints of the lines
    const a = fieldline.lineEndPointA.toThree()
    const b = fieldline.lineEndPointB.toThree()
    return this.lineProjection.planeSegment({
      start: Vector3.fromThree(
        a
          .clone()
          .applyMatrix4(this.params.Hcw.toThree())
          .normalize(),
      ),
      end: Vector3.fromThree(
        b
          .clone()
          .applyMatrix4(this.params.Hcw.toThree())  // transform point from world to new camera
          .normalize(),
      ),
      color: new Vector4(1.0, 1.0, 0.0, 1.0),
      lineWidth: 10,
    })
  })
}

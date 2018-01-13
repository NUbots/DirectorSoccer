import { createTransformer } from 'mobx'
import { computed } from 'mobx'
import { Mesh } from 'three'
import { MultiMaterial } from 'three'

import { geometryAndMaterial } from '../../utils'
import { HeadViewModel } from '../head/view_model'
import { LeftArmViewModel } from '../left_arm/view_model'
import { LeftLegViewModel } from '../left_leg/view_model'
import { LocalisationRobotModel } from '../model'
import { RightArmViewModel } from '../right_arm/view_model'
import { RightLegViewModel } from '../right_leg/view_model'

import * as BodyConfig from './config/body.json'

export class BodyViewModel {
  public constructor(private model: LocalisationRobotModel) {
  }

  public static of = createTransformer((model: LocalisationRobotModel): BodyViewModel => {
    return new BodyViewModel(model)
  })

  @computed
  public get body(): Mesh {
    const { geometry, materials } = this.bodyGeometryAndMaterial
    const mesh = new Mesh(geometry, new MultiMaterial(materials))
    mesh.position.set(0, 0, 0.096)
    mesh.add(this.head)
    mesh.add(this.leftArm)
    mesh.add(this.rightArm)
    mesh.add(this.leftLeg)
    mesh.add(this.rightLeg)
    mesh.rotation.x = Math.PI / 2
    mesh.rotation.y = Math.PI / 2
    return mesh
  }

  @computed
  private get bodyGeometryAndMaterial() {
    return geometryAndMaterial(BodyConfig, this.model.color)
  }

  @computed
  private get head() {
    return HeadViewModel.of(this.model).head
  }

  @computed
  private get leftArm() {
    return LeftArmViewModel.of(this.model).leftArm
  }

  @computed
  private get rightArm() {
    return RightArmViewModel.of(this.model).rightArm
  }

  @computed
  private get leftLeg() {
    return LeftLegViewModel.of(this.model).leftLeg
  }

  @computed
  private get rightLeg() {
    return RightLegViewModel.of(this.model).rightLeg
  }
}
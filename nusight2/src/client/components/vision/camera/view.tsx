import { action, computed } from 'mobx'
import { observer } from 'mobx-react'
import React, { Component } from 'react'
import { SwitchesMenu, SwitchesMenuOption } from '../../switches_menu/view'
import { Canvas, ObjectFit, Three } from '../../three/three'
import { CameraModel } from './model'
import styles from './styles.css'
import { CameraViewModel } from './view_model'



export type CameraViewProps = {
  model: CameraModel
}

@observer
export class CameraView extends Component<CameraViewProps> {
  render() {
    return (
      <div className={styles.camera}>
        <Three stage={this.stage} objectFit={this.objectFit} />
        <div className={styles.menu}>
          <SwitchesMenu dropdownMenuPosition="right" options={this.drawOptions} />
        </div>
      </div>
    )
  }

  @computed
  private get drawOptions(): SwitchesMenuOption[] {
    const { drawOptions } = this.props.model
    return [
      {
        label: 'Image',
        enabled: drawOptions.drawImage,
        toggle: action(() => (drawOptions.drawImage = !drawOptions.drawImage)),
      },
      {
        label: 'Visual Mesh',
        enabled: drawOptions.drawVisualmesh,
        toggle: action(() => (drawOptions.drawVisualmesh = !drawOptions.drawVisualmesh)),
      },
      {
        label: 'Distance',
        enabled: drawOptions.drawDistance,
        toggle: action(() => (drawOptions.drawDistance = !drawOptions.drawDistance)),
      },
      {
        label: 'Compass',
        enabled: drawOptions.drawCompass,
        toggle: action(() => (drawOptions.drawCompass = !drawOptions.drawCompass)),
      },
      {
        label: 'Horizon',
        enabled: drawOptions.drawHorizon,
        toggle: action(() => (drawOptions.drawHorizon = !drawOptions.drawHorizon)),
      },
      {
        label: 'Green Horizon',
        enabled: drawOptions.drawGreenhorizon,
        toggle: action(() => (drawOptions.drawGreenhorizon = !drawOptions.drawGreenhorizon)),
      },
      {
        label: 'Balls',
        enabled: drawOptions.drawBalls,
        toggle: action(() => (drawOptions.drawBalls = !drawOptions.drawBalls)),
      },
      {
        label: 'Goals',
        enabled: drawOptions.drawGoals,
        toggle: action(() => (drawOptions.drawGoals = !drawOptions.drawGoals)),
      },
      {
        label: 'FieldLines',
        enabled: drawOptions.drawFieldLines,
        toggle: action(() => (drawOptions.drawFieldLines = !drawOptions.drawFieldLines)),
      },
    ]
  }

  @computed.struct
  private get objectFit(): ObjectFit {
    const { width, height } = this.props.model.image
    return { type: 'contain', aspect: height / width }
  }

  private readonly stage = (canvas: Canvas) => {
    const cameraViewModel = CameraViewModel.of(canvas, this.props.model)
    return computed(() => [cameraViewModel.stage])
  }
}

import { useThree } from '@react-three/fiber'
import React from 'react'
import * as THREE from 'three'

import { Vector2 } from '../../../math/vector2'
import { Vector3 } from '../../../math/vector3'
import { Vector4 } from '../../../math/vector4'

import { Lens } from './model'
import fragmentShader from './shaders/line_projection.frag'
import vertexShader from './shaders/line_projection.vert'

export interface ConeSegment {
  /** The camera space central axis of the cone. */
  axis: Vector3
  /** The camera space vector pointing to the start of the segment. */
  start: Vector3
  /** The camera space vector pointing to the end of the segment. */
  end: Vector3
  /** The colour of the line to draw. */
  color?: Vector4
  /** The width of the line to draw on the screen in pixels. */
  lineWidth?: number
}

/** Draws a plane projected to infinity in world space. */
export const PlaneView = ({
  axis,
  color,
  lineWidth,
  lens,
}: {
  axis: Vector3
  color?: Vector4
  lineWidth?: number
  lens: Lens
}) => {
  // Pick an arbitrary orthogonal vector
  const start = Vector3.fromThree(
    !axis.x && !axis.y
      ? new THREE.Vector3(0, 1, 0)
      : new THREE.Vector3(-axis.y, axis.x, 0).normalize(),
  )
  return <ConeSegmentView segment={{ axis, start, end: start, color, lineWidth }} lens={lens} />
}

export const PlaneSegmentView = ({
  lens,
  ...segment
}: {
  axis?: Vector3
  start: Vector3
  end: Vector3
  color?: Vector4
  lineWidth?: number
  lens: Lens
}) => {
  return (
    <ConeSegmentView
      segment={{
        ...segment,
        axis:
          segment.axis ??
          Vector3.fromThree(
            new THREE.Vector3()
              .crossVectors(segment.start.toThree(), segment.end.toThree())
              .normalize(),
          ),
      }}
      lens={lens}
    />
  )
}

/** Draw a cone projected to infinity in world space. Only draws the positive cone, not the negative cone. */
export const ConeView = ({
  axis,
  radius,
  color,
  lineWidth,
  lens,
}: {
  axis: Vector3
  radius: number
  color?: Vector4
  lineWidth?: number
  lens: Lens
}) => {
  // Pick an arbitrary orthogonal vector
  const orth =
    !axis.x && !axis.y ? new Vector3(0, 1, 0) : new Vector3(-axis.y, axis.x, 0).normalize()
  // Rotate our axis by this radius to get a start
  const start = Vector3.fromThree(axis.toThree().applyAxisAngle(orth.toThree(), Math.acos(radius)))
  return <ConeSegmentView segment={{ axis, start, end: start, color, lineWidth }} lens={lens} />
}

export const ConeSegmentView = ({ segment, lens }: { segment: ConeSegment; lens: Lens }) => {
  const {
    projection,
    focalLength,
    centre = Vector2.of(),
    distortionCoeffecients = Vector2.of(),
  } = lens
  const { size } = useThree()
  return (
    <mesh>
      <planeGeometry args={[2, 2, 100]} />
      <rawShaderMaterial
        vertexShader={vertexShader}
        fragmentShader={fragmentShader}
        uniforms={{
          viewSize: { value: new THREE.Vector2(size.width, size.height) },
          projection: { value: projection },
          focalLength: { value: focalLength },
          centre: { value: centre.toThree() },
          k: { value: distortionCoeffecients.toThree() },
          axis: { value: segment.axis?.toThree() },
          start: { value: segment.start.toThree() },
          end: { value: segment.end.toThree() },
          color: { value: segment.color?.toThree() },
          lineWidth: { value: segment.lineWidth },
        }}
      />
    </mesh>
  )
}
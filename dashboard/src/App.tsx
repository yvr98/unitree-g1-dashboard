import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { Ros, Topic } from 'roslib'
import nipplejs from 'nipplejs'
import './App.css'

const ROSBRIDGE_URL = 'ws://localhost:9090'
const API_BASE_URL = 'http://localhost:8000'
const RETRY_INTERVAL_MS = 3000
const CMD_VEL_INTERVAL_MS = 100
const DEFAULT_SPEED = 0.5

type FsmState = 'IDLE' | 'STANDING' | 'WALKING' | 'EMERGENCY_STOP' | 'ERROR' | '—'

interface LocomotionStateMessage {
  state?: string
  previous_state?: string
  transition_reason?: string
  current_state?: string
  requested_state?: string
  is_transitioning?: boolean
  estop_active?: boolean
  transition_progress?: number
}

interface SafetyStatusMessage {
  is_safe?: boolean
  is_upright?: boolean
  estop_active?: boolean
  pitch?: number
  roll?: number
  current_alert?: string
}

interface RobotStateMessage {
  joint_position?: number[]
  imu_quaternion_wxyz?: number[]
  imu_rpy?: number[]
}

interface StateSnapshot {
  current_state?: string | null
  robot_state?: RobotStateMessage | null
  safety_status?: SafetyStatusMessage | null
  locomotion_state?: LocomotionStateMessage | null
}

interface CommandResponse {
  success: boolean
  accepted: boolean
  current_mode: string
  message: string
}

interface JointGroup {
  label: string
  start: number
  end: number
}

interface VelocityVector {
  linearX: number
  angularZ: number
}

const JOINT_GROUPS: JointGroup[] = [
  { label: 'Left Leg', start: 0, end: 5 },
  { label: 'Right Leg', start: 6, end: 11 },
  { label: 'Waist', start: 12, end: 14 },
  { label: 'Left Arm', start: 15, end: 21 },
  { label: 'Right Arm', start: 22, end: 28 },
]

const EMPTY_LOCOMOTION_STATE: LocomotionStateMessage = {
  current_state: '—',
  previous_state: '—',
  transition_reason: 'waiting_for_data',
  is_transitioning: false,
  estop_active: false,
  transition_progress: 0,
}

const EMPTY_SAFETY_STATUS: SafetyStatusMessage = {
  is_safe: false,
  is_upright: false,
  estop_active: false,
  pitch: 0,
  roll: 0,
  current_alert: 'waiting_for_data',
}

const EMPTY_ROBOT_STATE: RobotStateMessage = {
  joint_position: Array.from({ length: 29 }, () => 0),
  imu_quaternion_wxyz: [1, 0, 0, 0],
  imu_rpy: [0, 0, 0],
}

function staleLocomotionState(): LocomotionStateMessage {
  return {
    ...EMPTY_LOCOMOTION_STATE,
    current_state: '—',
    state: '—',
    previous_state: '—',
    transition_reason: 'disconnected',
    requested_state: '—',
    is_transitioning: false,
    estop_active: false,
    transition_progress: 0,
  }
}

function staleSafetyStatus(): SafetyStatusMessage {
  return {
    ...EMPTY_SAFETY_STATUS,
    is_safe: false,
    is_upright: false,
    estop_active: false,
    pitch: 0,
    roll: 0,
    current_alert: 'disconnected',
  }
}

function staleRobotState(): RobotStateMessage {
  return {
    ...EMPTY_ROBOT_STATE,
    joint_position: [...(EMPTY_ROBOT_STATE.joint_position ?? [])],
    imu_quaternion_wxyz: [...(EMPTY_ROBOT_STATE.imu_quaternion_wxyz ?? [])],
    imu_rpy: [...(EMPTY_ROBOT_STATE.imu_rpy ?? [])],
  }
}

function clamp(value: number, min: number, max: number) {
  return Math.max(min, Math.min(max, value))
}

function formatValue(value: number | undefined, digits = 2) {
  return typeof value === 'number' ? value.toFixed(digits) : '—'
}

function normalizeState(value: string | undefined | null): FsmState {
  if (value === 'IDLE' || value === 'STANDING' || value === 'WALKING' || value === 'EMERGENCY_STOP' || value === 'ERROR') {
    return value
  }

  return '—'
}

function getStateTone(state: FsmState) {
  switch (state) {
    case 'IDLE':
      return 'state-idle'
    case 'STANDING':
      return 'state-standing'
    case 'WALKING':
      return 'state-walking'
    case 'EMERGENCY_STOP':
      return 'state-estop'
    case 'ERROR':
      return 'state-error'
    default:
      return 'state-unknown'
  }
}

function quaternionToEulerDegrees(quaternion: number[] | undefined) {
  if (!quaternion || quaternion.length < 4) {
    return { pitch: 0, roll: 0, yaw: 0 }
  }

  const [w, x, y, z] = quaternion

  const sinrCosp = 2 * (w * x + y * z)
  const cosrCosp = 1 - 2 * (x * x + y * y)
  const roll = Math.atan2(sinrCosp, cosrCosp)

  const sinp = 2 * (w * y - z * x)
  const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * (Math.PI / 2) : Math.asin(sinp)

  const sinyCosp = 2 * (w * z + x * y)
  const cosyCosp = 1 - 2 * (y * y + z * z)
  const yaw = Math.atan2(sinyCosp, cosyCosp)

  const toDegrees = (value: number) => (value * 180) / Math.PI

  return {
    pitch: toDegrees(pitch),
    roll: toDegrees(roll),
    yaw: toDegrees(yaw),
  }
}

function App() {
  const [isConnected, setIsConnected] = useState(false)
  const [locomotionState, setLocomotionState] = useState<LocomotionStateMessage>(EMPTY_LOCOMOTION_STATE)
  const [safetyStatus, setSafetyStatus] = useState<SafetyStatusMessage>(EMPTY_SAFETY_STATUS)
  const [robotState, setRobotState] = useState<RobotStateMessage>(EMPTY_ROBOT_STATE)
  const [speedMultiplier, setSpeedMultiplier] = useState(DEFAULT_SPEED)
  const [commandPending, setCommandPending] = useState(false)
  const [commandFeedback, setCommandFeedback] = useState('')

  const joystickZoneRef = useRef<HTMLDivElement | null>(null)
  const rosRef = useRef<Ros | null>(null)
  const reconnectTimeoutRef = useRef<number | null>(null)
  const cmdVelTopicRef = useRef<Topic<VelocityCommand> | null>(null)
  const joystickVectorRef = useRef<VelocityVector>({ linearX: 0, angularZ: 0 })
  const joystickActiveRef = useRef(false)

  const currentState = normalizeState(locomotionState.current_state ?? locomotionState.state ?? null)
  const previousState = normalizeState(locomotionState.previous_state)
  const transitionReason = locomotionState.transition_reason ?? 'waiting_for_data'
  const isTransitioning = Boolean(locomotionState.is_transitioning)
  const joystickEnabled = isConnected && (currentState === 'STANDING' || currentState === 'WALKING')
  const estopActive = Boolean(safetyStatus.estop_active || locomotionState.estop_active)
  const showReset = estopActive || currentState === 'EMERGENCY_STOP' || currentState === 'ERROR'

  const imuAngles = useMemo(
    () => quaternionToEulerDegrees(robotState.imu_quaternion_wxyz),
    [robotState.imu_quaternion_wxyz],
  )

  const tiltIndicator = useMemo(() => {
    const radius = 34
    return {
      x: clamp((imuAngles.roll / 25) * radius, -radius, radius),
      y: clamp((imuAngles.pitch / 25) * radius, -radius, radius),
    }
  }, [imuAngles.pitch, imuAngles.roll])

  const jointGroups = useMemo(() => {
    const positions = robotState.joint_position ?? EMPTY_ROBOT_STATE.joint_position ?? []

    return JOINT_GROUPS.map((group) => ({
      ...group,
      joints: positions.slice(group.start, group.end + 1),
    }))
  }, [robotState.joint_position])

  const publishCmdVel = useCallback(
    (linearX: number, angularZ: number) => {
      const topic = cmdVelTopicRef.current
      if (!topic || !isConnected) {
        return
      }

      topic.publish({
        linear: { x: linearX, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angularZ },
      })
    },
    [isConnected],
  )

  const markDisconnected = useCallback(() => {
    setIsConnected(false)
    setLocomotionState(staleLocomotionState())
    setSafetyStatus(staleSafetyStatus())
    setRobotState(staleRobotState())
    setCommandPending(false)
    setCommandFeedback('Rosbridge disconnected. Retrying...')
    joystickActiveRef.current = false
    joystickVectorRef.current = { linearX: 0, angularZ: 0 }
    cmdVelTopicRef.current = null
  }, [])

  useEffect(() => {
    let disposed = false
    let activeTopics: Array<Topic<unknown>> = []

    const clearReconnect = () => {
      if (reconnectTimeoutRef.current !== null) {
        window.clearTimeout(reconnectTimeoutRef.current)
        reconnectTimeoutRef.current = null
      }
    }

    const scheduleReconnect = () => {
      if (disposed || reconnectTimeoutRef.current !== null) {
        return
      }

      reconnectTimeoutRef.current = window.setTimeout(() => {
        reconnectTimeoutRef.current = null
        connect()
      }, RETRY_INTERVAL_MS)
    }

    const cleanupTopics = () => {
      activeTopics.forEach((topic) => {
        topic.unsubscribe()
      })
      activeTopics = []
      cmdVelTopicRef.current = null
    }

    const connect = () => {
      clearReconnect()

      if (disposed) {
        return
      }

      cleanupTopics()
      rosRef.current?.close()

      const ros = new Ros({ url: ROSBRIDGE_URL })
      rosRef.current = ros

      ros.on('connection', () => {
        if (disposed || ros !== rosRef.current) {
          return
        }

        setIsConnected(true)
        setCommandFeedback('')

        const locomotionTopic = new Topic<LocomotionStateMessage>({
          ros,
          name: '/locomotion_state',
          messageType: 'g1_msgs/msg/LocomotionState',
        })

        const safetyTopic = new Topic<SafetyStatusMessage>({
          ros,
          name: '/safety_status',
          messageType: 'g1_msgs/msg/SafetyStatus',
        })

        const robotTopic = new Topic<RobotStateMessage>({
          ros,
          name: '/robot_state',
          messageType: 'g1_msgs/msg/RobotState',
          throttle_rate: 100,
        })

        const cmdVelTopic = new Topic<VelocityCommand>({
          ros,
          name: '/cmd_vel',
          messageType: 'geometry_msgs/msg/Twist',
        })

        locomotionTopic.subscribe((message) => {
          setLocomotionState((previous) => ({ ...previous, ...message }))
        })
        safetyTopic.subscribe((message) => {
          setSafetyStatus((previous) => ({ ...previous, ...message }))
        })
        robotTopic.subscribe((message) => {
          setRobotState((previous) => ({ ...previous, ...message }))
        })

        activeTopics = [locomotionTopic, safetyTopic, robotTopic]
        cmdVelTopicRef.current = cmdVelTopic
      })

      ros.on('error', () => {
        if (disposed || ros !== rosRef.current) {
          return
        }

        cleanupTopics()
        markDisconnected()
        scheduleReconnect()
      })

      ros.on('close', () => {
        if (disposed || ros !== rosRef.current) {
          return
        }

        cleanupTopics()
        markDisconnected()
        scheduleReconnect()
      })
    }

    connect()

    return () => {
      disposed = true
      clearReconnect()
      cleanupTopics()
      rosRef.current?.close()
      rosRef.current = null
    }
  }, [markDisconnected])

  useEffect(() => {
    const loadInitialState = async () => {
      try {
        const response = await fetch(`${API_BASE_URL}/api/state`)
        if (!response.ok) {
          return
        }

        const snapshot: StateSnapshot = await response.json()
        if (snapshot.locomotion_state) {
          setLocomotionState((previous) => ({ ...previous, ...snapshot.locomotion_state }))
        }
        if (snapshot.safety_status) {
          setSafetyStatus((previous) => ({ ...previous, ...snapshot.safety_status }))
        }
        if (snapshot.robot_state) {
          setRobotState((previous) => ({ ...previous, ...snapshot.robot_state }))
        }
      } catch (error) {
        console.debug('Initial API state fetch failed:', error)
      }
    }

    void loadInitialState()
  }, [])

  useEffect(() => {
    if (joystickEnabled) {
      return
    }

    joystickActiveRef.current = false
    joystickVectorRef.current = { linearX: 0, angularZ: 0 }
    publishCmdVel(0, 0)
  }, [joystickEnabled, publishCmdVel])

  useEffect(() => {
    if (!joystickEnabled) {
      return
    }

    const intervalId = window.setInterval(() => {
      if (!joystickActiveRef.current) {
        return
      }

      const vector = joystickVectorRef.current
      publishCmdVel(vector.linearX * speedMultiplier, vector.angularZ * speedMultiplier)
    }, CMD_VEL_INTERVAL_MS)

    return () => {
      window.clearInterval(intervalId)
    }
  }, [joystickEnabled, publishCmdVel, speedMultiplier])

  useEffect(() => {
    const zone = joystickZoneRef.current
    if (!zone) {
      return
    }

    const manager = nipplejs.create({
      zone,
      mode: 'static',
      position: { left: '50%', top: '50%' },
      color: { front: '#38bdf8', back: 'rgba(56, 189, 248, 0.16)' },
      size: 180,
      restOpacity: 0.75,
      threshold: 0.05,
      fadeTime: 120,
    })

    manager.on('move', (event) => {
      if (!joystickEnabled) {
        joystickActiveRef.current = false
        joystickVectorRef.current = { linearX: 0, angularZ: 0 }
        publishCmdVel(0, 0)
        return
      }

      const strength = clamp(event.data.force ?? 0, 0, 1)
      const vector = event.data.vector
      joystickActiveRef.current = true
      joystickVectorRef.current = {
        linearX: clamp(-(vector?.y ?? 0) * strength, -1, 1),
        angularZ: clamp(-(vector?.x ?? 0) * strength, -1, 1),
      }
    })

    manager.on('end', () => {
      joystickActiveRef.current = false
      joystickVectorRef.current = { linearX: 0, angularZ: 0 }
      publishCmdVel(0, 0)
    })

    return () => {
      manager.destroy()
    }
  }, [joystickEnabled, publishCmdVel])

  const sendCommand = async (command: 'stand_up' | 'sit_down' | 'reset') => {
    if (!isConnected) {
      setCommandFeedback('Rosbridge is disconnected.')
      return
    }

    setCommandPending(true)

    try {
      const response = await fetch(`${API_BASE_URL}/api/command`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ command }),
      })

      const payload = (await response.json()) as CommandResponse | { detail?: { message?: string } }
      if (!response.ok) {
        const detailMessage = 'detail' in payload ? payload.detail?.message : undefined
        setCommandFeedback(detailMessage ?? 'Command failed.')
        return
      }

      setCommandFeedback('message' in payload ? payload.message : 'Command sent.')
    } catch (error) {
      console.error('Command request failed:', error)
      setCommandFeedback('FastAPI is unavailable.')
    } finally {
      setCommandPending(false)
    }
  }

  return (
    <main className="dashboard-shell">
      <header className="top-bar card">
        <div className="connection-group">
          <span className={`status-dot ${isConnected ? 'online' : 'offline'}`} />
          <div>
            <p className="eyebrow">Connection</p>
            <strong>{isConnected ? 'Connected' : 'Disconnected'}</strong>
          </div>
        </div>
        <h1>Unitree G1 Dashboard</h1>
        <p className="top-bar-note">{ROSBRIDGE_URL}</p>
      </header>

      <section className="dashboard-grid">
        <div className="left-column">
          <section className={`card state-card ${getStateTone(currentState)}`}>
            <p className="eyebrow">Locomotion State</p>
            <div className="state-row">
              <span className="state-label">STATE</span>
              <strong className="state-value">{currentState}</strong>
            </div>
            <p className="state-subtext">Previous: {previousState}</p>
            <p className="state-subtext">Reason: {transitionReason}</p>
          </section>

          <section className="card control-card">
            <div className="button-row">
              <button
                type="button"
                onClick={() => void sendCommand('stand_up')}
                disabled={!isConnected || commandPending || isTransitioning || currentState !== 'IDLE'}
              >
                Stand Up
              </button>
              <button
                type="button"
                onClick={() => void sendCommand('sit_down')}
                disabled={!isConnected || commandPending || isTransitioning || currentState !== 'STANDING'}
              >
                Sit Down
              </button>
              {showReset ? (
                <button
                  type="button"
                  className="danger"
                  onClick={() => void sendCommand('reset')}
                  disabled={!isConnected || commandPending}
                >
                  Reset
                </button>
              ) : null}
            </div>
            <p className="feedback-line">{commandFeedback || 'Ready.'}</p>
          </section>

          <section className={`card joystick-card ${joystickEnabled ? '' : 'disabled'}`}>
            <div className="section-header">
              <p className="eyebrow">Virtual Joystick</p>
              <span className="mini-status">{joystickEnabled ? 'Enabled' : 'Stand to enable'}</span>
            </div>
            <div className="joystick-frame">
              <div ref={joystickZoneRef} className="joystick-zone" />
            </div>
            <div className="speed-row">
              <label htmlFor="speed-slider">Speed</label>
              <input
                id="speed-slider"
                type="range"
                min="0.1"
                max="1"
                step="0.1"
                value={speedMultiplier}
                onChange={(event) => setSpeedMultiplier(Number(event.target.value))}
              />
              <span>{speedMultiplier.toFixed(1)}</span>
            </div>
          </section>
        </div>

        <div className="right-column">
          <section className="card safety-card">
            <div className="section-header">
              <p className="eyebrow">Safety Status</p>
              <span className={`pill ${safetyStatus.is_safe ? 'good' : 'bad'}`}>
                {safetyStatus.is_safe ? 'Safe' : 'Unsafe'}
              </span>
            </div>
            <div className="indicator-row">
              <div className="indicator-item">
                <span className={`status-dot ${safetyStatus.is_safe ? 'online' : 'offline'}`} />
                <span>is_safe</span>
              </div>
              <div className="indicator-item">
                <span className={`status-dot ${safetyStatus.is_upright ? 'online' : 'offline'}`} />
                <span>is_upright</span>
              </div>
              <div className="indicator-item">
                <span className={`status-dot ${safetyStatus.estop_active ? 'estop' : 'idle'}`} />
                <span>estop_active</span>
              </div>
            </div>
            <div className="metric-grid compact">
              <div>
                <span>Pitch</span>
                <strong>{formatValue(safetyStatus.pitch)}</strong>
              </div>
              <div>
                <span>Roll</span>
                <strong>{formatValue(safetyStatus.roll)}</strong>
              </div>
            </div>
            <div className="alert-box">Alert: {safetyStatus.current_alert ?? '—'}</div>
          </section>

          <section className="card joints-card">
            <div className="section-header">
              <p className="eyebrow">Joint Positions</p>
              <span className="mini-status">29 joints</span>
            </div>
            <div className="joint-groups">
              {jointGroups.map((group) => (
                <div key={group.label} className="joint-group">
                  <h2>{group.label}</h2>
                  <div className="joint-grid">
                    {group.joints.map((value, index) => {
                      const jointIndex = group.start + index
                      return (
                        <div key={jointIndex} className="joint-cell">
                          <span>J{jointIndex}</span>
                          <strong>{formatValue(value)}</strong>
                        </div>
                      )
                    })}
                  </div>
                </div>
              ))}
            </div>
          </section>

          <section className="card imu-card">
            <div className="section-header">
              <p className="eyebrow">IMU Tilt</p>
              <span className="mini-status">Quaternion → Euler</span>
            </div>
            <div className="imu-layout">
              <div className="tilt-indicator">
                <div
                  className="tilt-dot"
                  style={{ transform: `translate(${tiltIndicator.x}px, ${tiltIndicator.y}px)` }}
                />
              </div>
              <div className="metric-grid">
                <div>
                  <span>Pitch</span>
                  <strong>{formatValue(imuAngles.pitch)}°</strong>
                </div>
                <div>
                  <span>Roll</span>
                  <strong>{formatValue(imuAngles.roll)}°</strong>
                </div>
                <div>
                  <span>Yaw</span>
                  <strong>{formatValue(imuAngles.yaw)}°</strong>
                </div>
              </div>
            </div>
          </section>
        </div>
      </section>
    </main>
  )
}

interface VelocityCommand {
  linear: { x: number; y: number; z: number }
  angular: { x: number; y: number; z: number }
}

export default App

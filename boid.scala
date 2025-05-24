package boid

import org.scalajs.dom
import org.scalajs.dom.{document, html}
import scala.util.Random
import vecxt.all.*
import vecxt.BoundsCheck.DoBoundsCheck.yes
import narr.*

val numBoids = 500

@main def main =
  dom.console.log("Boids simulation starting...")
  println("Boids simulation starting...")

  val positions = Matrix.zeros[Double](numBoids, 2)
  val velocities = Matrix.zeros[Double](numBoids, 2)
  val accelerations = Matrix.zeros[Double](numBoids, 2)
  val maxSpeed = 10
  val maxForce = 0.35
  val separationDistance = 25.0
  val alignmentDistance = 50.0
  val cohesionDistance = 50.0
  val width = dom.window.innerWidth
  val height = dom.window.innerHeight
  val canvas = document.createElement("canvas").asInstanceOf[html.Canvas]
  canvas.width = width.toInt
  canvas.height = height.toInt
  document.body.appendChild(canvas)

  val ctx = canvas.getContext("2d").asInstanceOf[dom.CanvasRenderingContext2D]
  ctx.fillStyle = "black"
  ctx.strokeStyle = "black"
  ctx.lineWidth = 1
  ctx.font = "16px Arial"
  ctx.textAlign = "center"
  ctx.textBaseline = "middle"

  // Initialize boids with random positions and velocities
  for (i <- 0 until numBoids) {
    positions((i, 0)) = Random.nextDouble() * width
    positions((i, 1)) = Random.nextDouble() * height
    velocities((i, 0)) = (Random.nextDouble() - 0.5) * maxSpeed
    velocities((i, 1)) = (Random.nextDouble() - 0.5) * maxSpeed
  }

  // Helper function to limit force magnitude
  def limitForce(fx: Double, fy: Double, maxForce: Double): (Double, Double) = {
    val magnitude = math.hypot(fx, fy)
    if (magnitude > maxForce && magnitude > 0) {
      (fx * maxForce / magnitude, fy * maxForce / magnitude)
    } else {
      (fx, fy)
    }
  }

  // Separation: steer to avoid crowding local flockmates
  def separate(boidIndex: Int): (Double, Double) = {
    var steerX = 0.0
    var steerY = 0.0
    var count = 0

    for (j <- 0 until numBoids if j != boidIndex) {
      val dx = positions((boidIndex, 0)) - positions((j, 0))
      val dy = positions((boidIndex, 1)) - positions((j, 1))
      val distance = math.hypot(dx, dy)

      if (distance > 0 && distance < separationDistance) {
        steerX += dx / distance
        steerY += dy / distance
        count += 1
      }
    }

    if (count > 0) {
      steerX /= count
      steerY /= count
      // Normalize and scale
      val magnitude = math.hypot(steerX, steerY)
      if (magnitude > 0) {
        steerX = (steerX / magnitude) * maxSpeed - velocities((boidIndex, 0))
        steerY = (steerY / magnitude) * maxSpeed - velocities((boidIndex, 1))
      }
    }

    limitForce(steerX, steerY, maxForce)
  }

  // Alignment: steer towards the average heading of neighbors
  def align(boidIndex: Int): (Double, Double) = {
    var sumVelX = 0.0
    var sumVelY = 0.0
    var count = 0

    for (j <- 0 until numBoids if j != boidIndex) {
      val dx = positions((boidIndex, 0)) - positions((j, 0))
      val dy = positions((boidIndex, 1)) - positions((j, 1))
      val distance = math.hypot(dx, dy)

      if (distance > 0 && distance < alignmentDistance) {
        sumVelX += velocities((j, 0))
        sumVelY += velocities((j, 1))
        count += 1
      }
    }

    if (count > 0) {
      sumVelX /= count
      sumVelY /= count
      // Normalize and scale
      val magnitude = math.hypot(sumVelX, sumVelY)
      if (magnitude > 0) {
        sumVelX = (sumVelX / magnitude) * maxSpeed - velocities((boidIndex, 0))
        sumVelY = (sumVelY / magnitude) * maxSpeed - velocities((boidIndex, 1))
      }
      limitForce(sumVelX, sumVelY, maxForce)
    } else {
      (0.0, 0.0)
    }
  }

  // Cohesion: steer to move toward the average position of neighbors
  def cohesion(boidIndex: Int): (Double, Double) = {
    var sumPosX = 0.0
    var sumPosY = 0.0
    var count = 0

    for (j <- 0 until numBoids if j != boidIndex) {
      val dx = positions((boidIndex, 0)) - positions((j, 0))
      val dy = positions((boidIndex, 1)) - positions((j, 1))
      val distance = math.hypot(dx, dy)

      if (distance > 0 && distance < cohesionDistance) {
        sumPosX += positions((j, 0))
        sumPosY += positions((j, 1))
        count += 1
      }
    }

    if (count > 0) {
      sumPosX /= count
      sumPosY /= count
      // Vector towards center of neighbors
      val targetX = sumPosX - positions((boidIndex, 0))
      val targetY = sumPosY - positions((boidIndex, 1))
      // Normalize and scale
      val magnitude = math.hypot(targetX, targetY)
      if (magnitude > 0) {
        val steerX =
          (targetX / magnitude) * maxSpeed - velocities((boidIndex, 0))
        val steerY =
          (targetY / magnitude) * maxSpeed - velocities((boidIndex, 1))
        limitForce(steerX, steerY, maxForce)
      } else {
        (0.0, 0.0)
      }
    } else {
      (0.0, 0.0)
    }
  }

  // Boundary wrapping
  def wrapBoundaries(boidIndex: Int): Unit = {
    if (positions((boidIndex, 0)) < 0) positions((boidIndex, 0)) = width
    if (positions((boidIndex, 0)) > width) positions((boidIndex, 0)) = 0
    if (positions((boidIndex, 1)) < 0) positions((boidIndex, 1)) = height
    if (positions((boidIndex, 1)) > height) positions((boidIndex, 1)) = 0
  }

  // Animation loop
  def animate(): Unit = {
    ctx.clearRect(0, 0, width, height)

    // Calculate accelerations for all boids
    for (i <- 0 until numBoids) {
      val (sepX, sepY) = separate(i)
      val (aliX, aliY) = align(i)
      val (cohX, cohY) = cohesion(i)

      // Weight the forces
      accelerations((i, 0)) = sepX * 1.5 + aliX * 1.0 + cohX * 1.0
      accelerations((i, 1)) = sepY * 1.5 + aliY * 1.0 + cohY * 1.0
    }

    // Update boids
    for (i <- 0 until numBoids) {
      // Update velocity and position
      velocities((i, 0)) = velocities((i, 0)) + accelerations((i, 0))
      velocities((i, 1)) = velocities((i, 1)) + accelerations((i, 1))

      // Limit speed
      val speed = math.hypot(velocities(i, 0), velocities(i, 1))
      if (speed > maxSpeed) {
        velocities((i, 0)) = velocities((i, 0)) * maxSpeed / speed
        velocities((i, 1)) = velocities((i, 1)) * maxSpeed / speed
      }

      positions((i, 0)) = positions((i, 0)) + velocities((i, 0))
      positions((i, 1)) = positions((i, 1)) + velocities((i, 1))

      // Wrap around boundaries
      wrapBoundaries(i)

      // Draw boid
      ctx.beginPath()
      ctx.arc(positions(i, 0), positions(i, 1), 5, 0, 2 * math.Pi)
      ctx.fill()
    }
  }

  // Call animate every 100ms
  dom.window.setInterval(() => animate(), 50)

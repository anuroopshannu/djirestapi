package com.rrc.djiControlServer

import android.Manifest
import android.content.Context
import android.content.Intent
import android.graphics.Bitmap
import androidx.appcompat.app.AppCompatActivity

import io.ktor.features.*
import io.ktor.http.*
import io.ktor.response.*
import io.ktor.server.engine.embeddedServer
import io.ktor.server.netty.Netty
import io.ktor.websocket.*
import android.net.wifi.WifiManager
import android.os.*
import android.text.format.Formatter
import android.util.Base64
import android.util.Log
import android.widget.TextView
import android.view.TextureView
import android.graphics.SurfaceTexture
import android.widget.Toast
import androidx.core.app.ActivityCompat
import dji.common.camera.SettingsDefinitions
import dji.common.error.DJIError
import dji.common.error.DJISDKError
import dji.common.flightcontroller.virtualstick.*
import dji.common.gimbal.GimbalMode
import dji.common.gimbal.Rotation
import dji.common.gimbal.RotationMode
import dji.common.util.CommonCallbacks
import dji.sdk.base.BaseComponent
import dji.sdk.base.BaseProduct
import dji.sdk.media.DownloadListener
import dji.sdk.media.FetchMediaTask
import dji.sdk.media.MediaManager
import dji.sdk.media.order.MediaRequest
import dji.sdk.products.Aircraft
import dji.sdk.sdkmanager.DJISDKInitEvent
import dji.sdk.sdkmanager.DJISDKManager
import io.ktor.application.*
import io.ktor.gson.*
import io.ktor.routing.*
import kotlinx.coroutines.Runnable
import kotlinx.coroutines.delay
import java.io.ByteArrayOutputStream
import java.nio.ByteBuffer
import kotlin.coroutines.resume
import kotlin.coroutines.suspendCoroutine
import kotlin.math.abs
import kotlin.math.min
import kotlin.math.pow
import dji.sdk.sdkmanager.LiveStreamManager
import dji.sdk.camera.VideoFeeder
import dji.sdk.codec.DJICodecManager
import dji.sdk.sdkmanager.LiveVideoBitRateMode
import dji.sdk.sdkmanager.LiveVideoResolution

//import fi.iki.elonen.NanoHTTPD
//import java.util.concurrent.LinkedBlockingQueue
//
//import android.os.Handler
//import android.os.Looper
//import android.os.SystemClock
//
//import android.view.PixelCopy
//import kotlinx.coroutines.CoroutineScope
//import kotlinx.coroutines.Dispatchers
//import kotlinx.coroutines.SupervisorJob
//import kotlinx.coroutines.launch
//import android.view.Surface

data class CommandCompleted(val completed: Boolean, val errorDescription: String?)

data class DroneState<T>(val state:T)

data class IMUState(val velX: Float, val velY: Float, val velZ: Float, val roll: Float, val pitch: Float, val yaw: Float)

data class VelocityCommand(val velX: Float, val velY: Float, val velZ: Float, val yawRate: Float)

class DJIControlException(message:String): Exception(message)

enum class VelocityProfile {
    CONSTANT, TRAPEZOIDAL, S_CURVE
}

enum class Direction {
    FORWARD, BACKWARD, LEFT, RIGHT, UP, DOWN, CLOCKWISE, COUNTER_CLOCKWISE
}

enum class ControlMode {
    POSITION, VELOCITY
}

class MainActivity : AppCompatActivity(), DJISDKManager.SDKManagerCallback {

    companion object {
        private const val PORT: Int = 8080
        private const val TAG = "MainActivity"
        private const val FLAG_CONNECTION_CHANGE = "dji_sdk_connection_change"
        private lateinit var mHandler: Handler
        private val REQUIRED_PERMISSION_LIST: Array<String> = arrayOf(
            Manifest.permission.VIBRATE,
            Manifest.permission.INTERNET,
            Manifest.permission.ACCESS_WIFI_STATE,
            Manifest.permission.WAKE_LOCK,
            Manifest.permission.ACCESS_COARSE_LOCATION,
            Manifest.permission.ACCESS_NETWORK_STATE,
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.CHANGE_WIFI_STATE,
            Manifest.permission.WRITE_EXTERNAL_STORAGE,
            Manifest.permission.BLUETOOTH,
            Manifest.permission.BLUETOOTH_ADMIN,
            Manifest.permission.READ_EXTERNAL_STORAGE,
            Manifest.permission.READ_PHONE_STATE,
        )
    }
    private lateinit var sdkManager: DJISDKManager

    private lateinit var ipText: TextView
    private lateinit var regText: TextView

    private lateinit var droneNameText: TextView
    private lateinit var batteryText: TextView

//    private lateinit var mjpegServer: MjpegStreamServer

    private var drone: Aircraft? = null

    private var imuStates = mutableListOf<IMUState?>()
    private var imuStatePostRunnable: Runnable? = null
    private var readIMUState = false

    private var controlMode = ControlMode.POSITION

    // Position Control Mode Constant
    private var maxSpeed = 0.2f // 20cm/s
    private var maxAngularSpeed = 30.0f // 30 deg/s
    private var maxAcceleration = 0.1f // 10 cm/s^2
    private var maxAngularAcceleration = 15.0f // 15 deg/s^2
    private var maxJerk = 0.2f // 20 cm/s^3
    private var maxAngularJerk = 30.0f // 30 deg/s^3
    private var flightCommandInterval = 40L // 40 ms = 25Hz
    private var velocityProfile: VelocityProfile = VelocityProfile.CONSTANT

    // Velocity Control Mode Constants
    private var velocityModeXVel = 0f
    private var velocityModeYVel = 0f
    private var velocityModeZVel = 0f
    private var velocityModeYawVel = 0f
    private var followingVelocityCommands = false
    private var velocityControlRunnable: Runnable? = null

    // Diagnostic fields to add at class level in MainActivity
//    private var lastFramePushTs: Long = 0L
//    private var producedFrameCounter: Long = 0L
//    private val producerLogIntervalMs = 5000L
//    private var lastProducerLogTs: Long = 0L
//
//    // Place these fields inside your MainActivity class (adjust names if needed)
//    private val TAG_INSTR = "ProducerInstr"
//    private val handler = Handler(Looper.getMainLooper())
//    private val watchdogHandler = Handler(Looper.getMainLooper())
//
//    private var expectedNextRunMs: Long = SystemClock.elapsedRealtime()
//    private val PRODUCER_INTERVAL_MS: Long = 250L // match your producer schedule (ms)
//    private val WATCHDOG_INTERVAL_MS: Long = 1000L
//    private val GAP_THRESHOLD_MS: Long = 2000L // threshold to consider producer stalled
//
//    // Producer state tracking (ensure pushPreviewToMjpeg calls markFrameProduced() on success)
//    @Volatile
//    private var lastFramePushTs: Long = System.currentTimeMillis()
//    private var producedFrameCounter: Long = 0L
//
//    // --- PixelCopy producer + background JPEG encode (paste into MainActivity) ---
//
//    // Coroutine scope for JPEG encoding (IO)
//    private val jpegEncodingScope = CoroutineScope(Dispatchers.IO + SupervisorJob())
//
//    // Producer loop handler (use the class-level handler you've already defined)
//    private var pixelCopyProducerPosted = false
//    private var pixelCopyIntervalMs: Long = 250L // default 250ms -> 4fps, tune as needed
//
//    // Start/stop helpers. Call startPixelCopyProducerLoop(fps) instead of scheduling your old handler.
//    private fun startPixelCopyProducerLoop(fps: Int = 4) {
//        stopPixelCopyProducerLoop()
//        pixelCopyIntervalMs = (1000L / fps).coerceAtLeast(40L)
//        pixelCopyProducerPosted = true
//        handler.post(pixelCopyProducerRunnable)
//        Log.i(TAG, "PixelCopy producer loop started fps=$fps intervalMs=$pixelCopyIntervalMs")
//    }
//
//    private fun stopPixelCopyProducerLoop() {
//        pixelCopyProducerPosted = false
//        handler.removeCallbacks(pixelCopyProducerRunnable)
//        Log.i(TAG, "PixelCopy producer loop stopped")
//    }
//
//    // The runnable that triggers captures
//    private val pixelCopyProducerRunnable = object : Runnable {
//        override fun run() {
//            if (!pixelCopyProducerPosted) return
//
//            if (!::textureView.isInitialized || !textureView.isAvailable) {
//                Log.w(TAG, "PixelCopy producer: textureView not ready, skipping tick")
//            } else {
//                if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.O) {
//                    pushPreviewToMjpeg_pixelcopy()
//                } else {
//                    // older devices fallback to existing synchronous path (keeps behaviour)
//                    pushPreviewToMjpeg_fallback_bitmap()
//                }
//            }
//
//            handler.postDelayed(this, pixelCopyIntervalMs)
//        }
//    }
//
//    // PixelCopy-based capture: copies TextureView -> Bitmap, then compresses on IO thread and offers to mjpegServer
//    private fun pushPreviewToMjpeg_pixelcopy() {
//        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.O) {
//            // fallback to your existing bitmap capture if needed
//            pushPreviewToMjpeg_fallback_bitmap()
//            return
//        }
//
//        val surfaceTexture = textureView.surfaceTexture
//        if (surfaceTexture == null) {
//            Log.w(TAG, "PixelCopy: textureView.surfaceTexture is null, skipping PixelCopy")
//            return
//        }
//
//        val bmp = Bitmap.createBitmap(textureView.width, textureView.height, Bitmap.Config.ARGB_8888)
//        val surface = Surface(surfaceTexture) // temporary wrapper for PixelCopy
//        val mainHandler = Handler(Looper.getMainLooper())
//
//        try {
//            PixelCopy.request(surface, /* srcRect = */ null, bmp, PixelCopy.OnPixelCopyFinishedListener { copyResult ->
//                try {
//                    if (copyResult == PixelCopy.SUCCESS) {
//                        // offload JPEG compression / offering to IO (example using your scope)
//                        jpegEncodingScope.launch {
//                            try {
//                                val start = SystemClock.elapsedRealtime()
//                                val out = ByteArrayOutputStream()
//                                val ok = bmp.compress(Bitmap.CompressFormat.JPEG, 60, out)
//                                val dur = SystemClock.elapsedRealtime() - start
//                                if (ok) {
//                                    val bytes = out.toByteArray()
//                                    val offered = mjpegServer.offerFrame(bytes)
//                                    if (offered) markFrameProduced()
//                                    Log.d(TAG, "PixelCopy: encoded=${bytes.size} offered=$offered compressMs=$dur")
//                                } else {
//                                    Log.w(TAG, "PixelCopy: compress returned false (compressMs=$dur)")
//                                }
//                            } catch (e: Exception) {
//                                Log.e(TAG, "PixelCopy: encode/offer exception", e)
//                            } finally {
//                                try { bmp.recycle() } catch (_: Exception) {}
//                            }
//                        }
//                    } else {
//                        Log.w(TAG, "PixelCopy: request failed result=$copyResult")
//                        try { bmp.recycle() } catch (_: Exception) {}
//                    }
//                } finally {
//                    // release the temporary Surface wrapper to avoid native resource leak
//                    try { surface.release() } catch (_: Exception) {}
//                }
//            }, mainHandler)
//        } catch (e: Exception) {
//            Log.e(TAG, "PixelCopy: request threw", e)
//            try { bmp.recycle() } catch (_: Exception) {}
//            try { surface.release() } catch (_: Exception) {}
//        }
//    }
//
//    // Fallback path that uses textureView.bitmap synchronously but still compresses on IO
//    private fun pushPreviewToMjpeg_fallback_bitmap() {
//        try {
//            val bmp = textureView.bitmap
//            if (bmp == null) {
//                Log.w(TAG, "Fallback: textureView.bitmap returned null")
//                return
//            }
//            jpegEncodingScope.launch {
//                try {
//                    val start = SystemClock.elapsedRealtime()
//                    val out = ByteArrayOutputStream()
//                    val ok = bmp.compress(Bitmap.CompressFormat.JPEG, 60, out)
//                    val dur = SystemClock.elapsedRealtime() - start
//                    if (!ok) {
//                        Log.w(TAG, "Fallback IO: compress returned false (dur=${dur}ms)")
//                    } else {
//                        val bytes = out.toByteArray()
//                        val offered = mjpegServer.offerFrame(bytes)
//                        if (offered) markFrameProduced() else Log.w(TAG, "Fallback IO: offerFrame returned false")
//                        Log.d(TAG, "Fallback IO: encoded=${bytes.size} offered=$offered durMs=$dur")
//                    }
//                } catch (e: Exception) {
//                    Log.e(TAG, "Fallback IO: compress/offer exception", e)
//                } finally {
//                    try { bmp.recycle() } catch (_: Exception) {}
//                }
//            }
//        } catch (e: Exception) {
//            Log.e(TAG, "Fallback: textureView.bitmap exception", e)
//        }
//    }
//
//    // Runnable that instruments scheduling and capture duration. Replace your existing scheduled runnable with this.
//    private val instrumentedProducerRunnable = object : Runnable {
//        override fun run() {
//            val now = SystemClock.elapsedRealtime()
//            val drift = now - expectedNextRunMs
//            Log.d(TAG_INSTR, "ProducerRunnable.start now=$now expected=$expectedNextRunMs driftMs=$drift")
//
//            val runStart = SystemClock.elapsedRealtime()
//            val produced = try {
//                pushPreviewToMjpeg()
//            } catch (e: Exception) {
//                Log.e(TAG_INSTR, "ProducerRunnable: pushPreviewToMjpeg threw", e)
//                false
//            }
//            val runEnd = SystemClock.elapsedRealtime()
//
//            Log.d(TAG_INSTR, "ProducerRunnable.end now=$runEnd runMs=${runEnd - runStart} produced=$produced")
//
//            expectedNextRunMs += PRODUCER_INTERVAL_MS
//            if (expectedNextRunMs < SystemClock.elapsedRealtime()) {
//                expectedNextRunMs = SystemClock.elapsedRealtime() + PRODUCER_INTERVAL_MS
//            }
//            handler.postDelayed(this, PRODUCER_INTERVAL_MS)
//        }
//    }
//
//    // Watchdog that checks lastFramePushTs and dumps stacks if a gap is detected
//    private val watchdogRunnable = object : Runnable {
//        override fun run() {
//            val gap = System.currentTimeMillis() - lastFramePushTs
//            Log.d(TAG_INSTR, "WATCHDOG: lastFramePush gapMs=$gap")
//            if (gap > GAP_THRESHOLD_MS) {
//                Log.w(TAG_INSTR, "WATCHDOG: gap exceeded ${GAP_THRESHOLD_MS}ms (gap=${gap}ms) - dumping stacks")
//                dumpAllStacks("producer_gap")
//            }
//            watchdogHandler.postDelayed(this, WATCHDOG_INTERVAL_MS)
//        }
//    }
//
//    // Call this from pushPreviewToMjpeg when a frame was successfully enqueued/offered
//    private fun markFrameProduced() {
//        lastFramePushTs = System.currentTimeMillis()
//        producedFrameCounter++
//        Log.d(TAG_INSTR, "markFrameProduced: ts=$lastFramePushTs producedCount=$producedFrameCounter")
//    }
//
//    // Dumps all Java thread stacks to logcat. (Optionally write to a file here for later retrieval.)
//    private fun dumpAllStacks(reason: String) {
//        try {
//            val sb = StringBuilder()
//            sb.append("STACK_DUMP reason=$reason time=${System.currentTimeMillis()}\n")
//            val all = Thread.getAllStackTraces()
//            for ((thread, stack) in all) {
//                sb.append("Thread \"${thread.name}\" id=${thread.id} state=${thread.state}\n")
//                for (el in stack) {
//                    sb.append("\t$el\n")
//                }
//            }
//            Log.w(TAG_INSTR, sb.toString())
//            // Optional: persist sb.toString() to a file for later analysis.
//        } catch (e: Exception) {
//            Log.e(TAG_INSTR, "dumpAllStacks failed", e)
//        }
//    }
//
//    // Helper functions to start/stop instrumentation. Call startInstrumentation() after your MJPEG server and producer loop are created.
//    private fun startInstrumentation() {
//        expectedNextRunMs = SystemClock.elapsedRealtime() + PRODUCER_INTERVAL_MS
//        handler.post(instrumentedProducerRunnable)
//        watchdogHandler.postDelayed(watchdogRunnable, WATCHDOG_INTERVAL_MS)
//        Log.i(TAG_INSTR, "Instrumentation started (interval=${PRODUCER_INTERVAL_MS}ms watchdog=${WATCHDOG_INTERVAL_MS}ms gapThreshold=${GAP_THRESHOLD_MS}ms)")
//    }
//
//    // Call this in onPause/onDestroy to stop logging and avoid leaks
//    private fun stopInstrumentation() {
//        handler.removeCallbacks(instrumentedProducerRunnable)
//        watchdogHandler.removeCallbacks(watchdogRunnable)
//        Log.i(TAG_INSTR, "Instrumentation stopped")
//    }
//
//    // Change signature and body of your producer to return boolean
//    private fun pushPreviewToMjpeg(): Boolean {
//        android.util.Log.w(TAG, "Producer: initialized")
//
//        try {
//            if (!::textureView.isInitialized || !textureView.isAvailable) {
//                android.util.Log.w(TAG, "Producer: textureView not initialized/available")
//                return false
//            }
//
//            val captureStart = SystemClock.elapsedRealtime()
//            val bitmap = try {
//                textureView.bitmap
//            } catch (e: Exception) {
//                android.util.Log.e(TAG, "Producer: exception reading textureView.bitmap", e)
//                return false
//            }
//            val captureMs = SystemClock.elapsedRealtime() - captureStart
//
//            if (bitmap == null) {
//                android.util.Log.w(TAG, "Producer: textureView.bitmap returned null (captureMs=$captureMs)")
//                return false
//            }
//
//            val out = ByteArrayOutputStream()
//            val compressStart = SystemClock.elapsedRealtime()
//            val compressOk = try {
//                bitmap.compress(Bitmap.CompressFormat.JPEG, 60, out)
//            } catch (e: Exception) {
//                android.util.Log.e(TAG, "Producer: Bitmap.compress threw", e)
//                false
//            }
//            val compressMs = SystemClock.elapsedRealtime() - compressStart
//
//            if (!compressOk) {
//                android.util.Log.w(TAG, "Producer: Bitmap.compress returned false (compressMs=$compressMs)")
//                bitmap.recycle()
//                return false
//            }
//
//            val bytes = out.toByteArray()
//            val offered = mjpegServer.offerFrame(bytes)
//            val qSize = mjpegServer.queueSize()
//
//            if (offered) {
//                markFrameProduced()
//            } else {
//                android.util.Log.w(TAG, "Producer: offerFrame returned false (queueSize=$qSize)")
//            }
//
//            android.util.Log.d(TAG, "Producer: encoded=${bytes.size} offered=$offered queueSize=$qSize captureMs=$captureMs compressMs=$compressMs")
//            bitmap.recycle()
//            return offered
//        } catch (e: Exception) {
//            android.util.Log.e(TAG, "Producer: unexpected error", e)
//            return false
//        }
//        android.util.Log.w(TAG, "Producer: completed")
//
//    }

    //livestream constants
    private var codecManager: DJICodecManager? = null
    private lateinit var textureView: TextureView
    private val videoDataListener = VideoFeeder.VideoDataListener { videoBuffer, size ->
        codecManager?.sendDataToDecoder(videoBuffer, size)
    }


    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        ipText = findViewById(R.id.ipText)
        regText = findViewById(R.id.regText)
        droneNameText = findViewById(R.id.droneNameText)
        batteryText = findViewById(R.id.batteryText)

        // Visible TextureView for DJI video feed (not hidden!)
        textureView = findViewById(R.id.dji_video_texture)

        displayIP()
        startServer()

        if(Build.VERSION.SDK_INT >= Build.VERSION_CODES.M)
            ActivityCompat.requestPermissions(this, REQUIRED_PERMISSION_LIST, 1)

        mHandler = Handler(Looper.getMainLooper())
        sdkManager = DJISDKManager.getInstance()
        sdkManager.setCallbackRunInUIThread(true)

        regText.text = getString(R.string.registering)

        sdkManager.registerApp(this, this)

        // Start MJPEG server on port 8090 in a background thread
//        mjpegServer = MjpegStreamServer(8090)
//        Thread { mjpegServer.start() }.start()

//        val heartbeatHandler = Handler(Looper.getMainLooper())
//        val heartbeatRunnable = object : Runnable {
//            override fun run() {
//                Log.v(HEARTBEAT, "HEARTBEAT main alive ts=${SystemClock.elapsedRealtime()}")
//                heartbeatHandler.postDelayed(this, 1000L)
//            }
//        }
//        handler.post(heartbeatRunnable)
//        startPixelCopyProducerLoop(fps = 4) // tune fps (4 used as example)
//        startInstrumentation()
    }

    // Embedded Server Functions
    private fun displayIP() {
        val wifiManager = applicationContext.getSystemService(Context.WIFI_SERVICE) as WifiManager
        val ipString = Formatter.formatIpAddress(wifiManager.connectionInfo.ipAddress)
        ipText.text = getString(R.string.ip_format, ipString, PORT)
    }

    private fun startServer() {
        embeddedServer(Netty, PORT) {
            install(WebSockets)
            install(CallLogging)
            install(ContentNegotiation) {
                gson {  }
            }

            routing {
                meta()
                getState()
                setState()
                cameraControl()
                takeoffAndLandControl()
                velocityControl()
                throttleControl()
                yawControl()
                rollPitchControl()
                livestreamControl()
            }
        }.start(wait = false)
    }

    // Server Routes
    private fun Route.meta() {
        get("/") {
            call.respondText ( text="Connected", contentType = ContentType.Text.Plain )
        }

        get("/reboot") {
            if(drone != null) {
                if(drone!!.flightController != null) {
                    val rebootError = suspendCoroutine<DJIError?> { cont ->
                        drone!!.flightController!!.reboot { error ->
                            cont.resume(error)
                        }
                    }

                    if(rebootError != null)
                        call.respond(CommandCompleted(false, "Reboot Error: " + rebootError.description))

                    call.respond(CommandCompleted(true, null))
                }
                else
                    call.respond(CommandCompleted(false, "Flight Controller Not available"))
            }
            else
                call.respond(CommandCompleted(false, "Drone not Available"))
        }

        get("/startCollectingIMUState/{interval}") {
            if(drone != null) {

                try {
                    val imuReadInterval = if(call.parameters["interval"] != null)
                        call.parameters["interval"]!!.toLong()
                    else
                        1000L // Default once a second

                    val handler  = Handler(Looper.getMainLooper())

                    readIMUState = true

                    imuStatePostRunnable = object: Runnable {
                        override fun run() {
                            val currIMUState = drone?.flightController?.state
                            val currFilteredState = if(currIMUState != null)
                                IMUState(currIMUState.velocityX, currIMUState.velocityY, currIMUState.velocityZ,
                                    currIMUState.attitude.roll.toFloat(), currIMUState.attitude.pitch.toFloat(), currIMUState.attitude.yaw.toFloat())
                            else
                                null
                            imuStates.add(currFilteredState)
                            if(readIMUState)
                                handler.postDelayed(this, imuReadInterval)
                        }
                    }

                    handler.postDelayed(imuStatePostRunnable as Runnable, imuReadInterval)

                    call.respond(CommandCompleted(true, null))
                }
                catch(e:NumberFormatException) {
                    call.respond(CommandCompleted(false, "Invalid value for interval. Must be a positive integer."))
                }
                catch(e: Exception) {
                    call.respond(CommandCompleted(false, e.message))
                }
            }
            else
                call.respond(CommandCompleted(false, "Drone Not Available"))
        }

        get("/stopCollectingIMUState") {
            if(drone != null) {
                try {
                    readIMUState = false
                    val handler = Handler(Looper.getMainLooper())
                    handler.removeCallbacks(imuStatePostRunnable as Runnable)
                    call.respond(CommandCompleted(true, null))
                }
                catch(e: Exception) {
                    call.respond(CommandCompleted(false, e.message))
                }
            }
            else
                call.respond(CommandCompleted(false, "Drone Not Available"))
        }

        get("/getCurrentIMUState") {
            if(drone != null) {
                try {
                    val currIMUState = drone?.flightController?.state
                    val currFilteredState = if(currIMUState != null)
                        IMUState(currIMUState.velocityX, currIMUState.velocityY, currIMUState.velocityZ,
                            currIMUState.attitude.roll.toFloat(), currIMUState.attitude.pitch.toFloat(), currIMUState.attitude.yaw.toFloat())
                    else
                        null

                    if(currFilteredState == null)
                        call.respond(CommandCompleted(false, "Unable to fetch IMU data."))
                    else
                        call.respond(DroneState(currFilteredState))
                }
                catch (e: Exception) {
                    call.respond(CommandCompleted(false, e.message))
                }
            }
            else
                call.respond(CommandCompleted(false, "Drone Not Available"))
        }

        get("/getCollectedIMUStates") {
            try {
                call.respond(DroneState(imuStates))
            }
            catch(e: Exception) {
                call.respond(CommandCompleted(false, e.message))
            }
        }

        get("/clearCollectedIMUStates") {
            try {
                imuStates.clear()
                call.respond(CommandCompleted(true, null))
            }
            catch(e: Exception) {
                call.respond(CommandCompleted(false, e.message))
            }
        }
    }

    private fun Route.getState() {

        get("/isLandingProtectionEnabled") {
            if(drone != null) {
                val isLandingProtectionEnabled = suspendCoroutine { cont ->
                    drone?.flightController?.flightAssistant?.getLandingProtectionEnabled(object: CommonCallbacks.CompletionCallbackWith<Boolean> {
                        override fun onSuccess(p0: Boolean?) {
                            cont.resume(p0)
                        }

                        override fun onFailure(p0: DJIError?) {
                            cont.resume(null)
                        }

                    })
                }
                if(isLandingProtectionEnabled != null)
                    call.respond(DroneState<Boolean?>(isLandingProtectionEnabled))
                else
                    call.respond(CommandCompleted(false, "Unable to fetch state"))
            }
            else
                call.respond(CommandCompleted(false, "Drone Not Available"))

        }

        get("/isVirtualStickControlEnabled") {
            if(drone != null)
                call.respond(DroneState(drone?.flightController?.isVirtualStickControlModeAvailable))
            else
                call.respond(CommandCompleted(false, "Drone Not Available"))

        }

        get("/getMaxSpeed") {
            call.respond(DroneState(maxSpeed))
        }

        get("/getMaxAngularSpeed") {
            call.respond(DroneState(maxAngularSpeed))
        }

        get("/getVelocityProfile") {
            val profileName = when(velocityProfile) {
                VelocityProfile.CONSTANT -> "CONSTANT"
                VelocityProfile.TRAPEZOIDAL -> "TRAPEZOIDAL"
                VelocityProfile.S_CURVE -> "S_CURVE"
            }
            call.respond(DroneState(profileName))
        }

        get("/getControlMode") {
            val controlModeName = when(controlMode) {
                ControlMode.POSITION -> "POSITION"
                ControlMode.VELOCITY -> "VELOCITY"
            }

            call.respond(DroneState(controlModeName))
        }

        get("/getHeading") {
            if(drone != null) {
                try{
                    call.respond(DroneState(drone?.flightController?.compass!!.heading))
                }
                catch(e: AssertionError) {
                    call.respond(CommandCompleted(false, "Cannot find compass component. Unable to fetch heading"))
                }
            }
            else
                call.respond(CommandCompleted(false, "Drone Not Available"))
        }

        get("/getAltitude") {
            if(drone == null)
                return@get call.respond(CommandCompleted(false, "Drone Not Available"))

            if(drone!!.flightController == null)
                return@get call.respond(CommandCompleted(false, "Flight Controller Unavailable"))

            if(drone!!.flightController!!.state.isUltrasonicBeingUsed)
                return@get call.respond(DroneState(drone!!.flightController!!.state.ultrasonicHeightInMeters))
            else
                return@get call.respond(CommandCompleted(false, "Ultrasonic Sensor Not Being Used Currently"))

        }

    }

    private fun Route.setState() {

        get("/enableLandingProtection") {
            if(drone != null) {
                val stateChangeError = suspendCoroutine<DJIError?> { cont ->
                    drone?.flightController?.flightAssistant?.setLandingProtectionEnabled(true) { error ->
                        cont.resume(error)
                    }
                }

                call.respond(CommandCompleted(stateChangeError == null, stateChangeError?.description))
            }
            else
                call.respond(CommandCompleted(false, "Drone Not Available"))

        }

        get("/disableLandingProtection") {
            if(drone != null) {
                val stateChangeError = suspendCoroutine<DJIError?> { cont ->
                    drone?.flightController?.flightAssistant?.setLandingProtectionEnabled(false) { error ->
                        cont.resume(error)
                    }
                }

                call.respond(CommandCompleted(stateChangeError == null, stateChangeError?.description))
            }
            else
                call.respond(CommandCompleted(false, "Drone Not Available"))

        }

        get("/setMaxSpeed/{speed}") {
            if(call.parameters["speed"] != null) {
                try {
                    val speed = call.parameters["speed"]!!.toFloat()

                    if(speed <= 0)
                        throw NumberFormatException("Speed must be a positive float")

                    maxSpeed = speed

                    call.respond(CommandCompleted(true, null))
                }
                catch(e: Exception){
                    call.respond(CommandCompleted(false, e.message))
                }
            }
            else
                call.respond(CommandCompleted(false, "No speed provided"))
        }

        get("/setMaxAngularSpeed/{speed}") {
            if(call.parameters["speed"] != null) {
                try {
                    val speed = call.parameters["speed"]!!.toFloat()

                    if(speed <= 0)
                        throw NumberFormatException("Speed must be a positive float")

                    maxAngularSpeed = speed

                    call.respond(CommandCompleted(true, null))
                }
                catch(e: Exception){
                    call.respond(CommandCompleted(false, e.message))
                }
            }
            else
                call.respond(CommandCompleted(false, "No speed provided"))
        }

        get("/setVelocityProfile/{profile}") {
            if(call.parameters["profile"] != null) {
                val profile = call.parameters["profile"].toString().uppercase()
                var validProfile = true
                when(profile) {
                    "CONSTANT" -> velocityProfile = VelocityProfile.CONSTANT
                    "TRAPEZOIDAL" -> velocityProfile = VelocityProfile.TRAPEZOIDAL
                    "S_CURVE" -> velocityProfile = VelocityProfile.S_CURVE
                    else -> validProfile = false
                }

                if(validProfile)
                    call.respond(CommandCompleted(true, null))
                else
                    call.respond(CommandCompleted(false, "Profile must be either 'CONSTANT', 'TRAPEZOIDAL' or 'S_CURVE'"))
            }
            else
                call.respond(CommandCompleted(false, "No profile provided"))
        }

        get("/setControlMode/{mode}") {
            if(call.parameters["mode"] != null) {

                try {
                    if(followingVelocityCommands)
                        throw DJIControlException("Cannot change control mode while velocity commands are being followed. First stop velocity control and then try again.")
                    val mode = call.parameters["mode"].toString().uppercase()
                    var validMode = true
                    when(mode) {
                        "POSITION" -> controlMode = ControlMode.POSITION
                        "VELOCITY" -> controlMode = ControlMode.VELOCITY
                        else -> validMode = false
                    }

                    if(validMode)
                        call.respond(CommandCompleted(true, null))
                    else
                        call.respond(CommandCompleted(false, "Profile must either be 'POSITION' or 'VELOCITY'"))
                }
                catch(e: DJIControlException){
                    call.respond(CommandCompleted(false, e.message))
                }
            }
            else
                call.respond(CommandCompleted(false, "No control mode provided"))
        }
    }

    private fun Route.cameraControl() {

        get("/captureShot") {

            // Availability Checks
            if(drone == null)
                return@get call.respond(CommandCompleted(false, "Drone not Available"))
            if(drone!!.camera == null)
                return@get call.respond(CommandCompleted(false, "Camera not Available"))
            if(drone!!.camera!!.mediaManager == null)
                return@get call.respond(CommandCompleted(false, "Media Manager Not Available"))
            if(!drone!!.camera!!.isFlatCameraModeSupported)
                return@get call.respond(CommandCompleted(false, "Only Flat-Camera Mode is supported"))

            // Get out of playback mode if in it
            suspendCoroutine<DJIError?> { cont ->
                drone!!.camera!!.exitPlayback { error ->
                    cont.resume(error)
                }
            }

            val flatModeError = suspendCoroutine<DJIError?> { cont ->
                drone!!.camera!!.setFlatMode(SettingsDefinitions.FlatCameraMode.PHOTO_SINGLE) { error ->
                    cont.resume(error)
                }
            }

            if(flatModeError != null)
                return@get call.respond(CommandCompleted(false, "Error in setting flat mode: " + flatModeError.description))

            val singleShotError = suspendCoroutine<DJIError?> { cont ->
                drone!!.camera!!.startShootPhoto { error ->
                    cont.resume(error)
                }
            }

            if(singleShotError != null)
                return@get call.respond(CommandCompleted(false, "Error in taking single shot" +  singleShotError.description))

            return@get call.respond(CommandCompleted(true, null))

        }

        get ("/startVideoRecording") {
            // Availability Checks
            if(drone == null)
                return@get call.respond(CommandCompleted(false, "Drone not Available"))
            if(drone!!.camera == null)
                return@get call.respond(CommandCompleted(false, "Camera not Available"))
            if(drone!!.camera!!.mediaManager == null)
                return@get call.respond(CommandCompleted(false, "Media Manager Not Available"))
            if(!drone!!.camera!!.isFlatCameraModeSupported)
                return@get call.respond(CommandCompleted(false, "Only Flat-Camera Mode is supported"))

            // Get out of playback mode if in it
            suspendCoroutine<DJIError?> { cont ->
                drone!!.camera!!.exitPlayback { error ->
                    cont.resume(error)
                }
            }

            val flatModeError = suspendCoroutine<DJIError?> { cont ->
                drone!!.camera!!.setFlatMode(SettingsDefinitions.FlatCameraMode.VIDEO_NORMAL) { error ->
                    cont.resume(error)
                }
            }

            if(flatModeError != null)
                return@get call.respond(CommandCompleted(false, "Error in setting flat mode: " + flatModeError.description))

            val startVideoError = suspendCoroutine<DJIError?> { cont ->
                drone!!.camera!!.startRecordVideo { error ->
                    cont.resume(error)
                }
            }

            if(startVideoError != null)
                return@get call.respond(CommandCompleted(false, "Error in starting video recording" +  startVideoError.description))

            return@get call.respond(CommandCompleted(true, null))
        }

        get("/stopVideoRecording") {
            // Availability Checks
            if(drone == null)
                return@get call.respond(CommandCompleted(false, "Drone not Available"))
            if(drone!!.camera == null)
                return@get call.respond(CommandCompleted(false, "Camera not Available"))
            if(drone!!.camera!!.mediaManager == null)
                return@get call.respond(CommandCompleted(false, "Media Manager Not Available"))
            if(!drone!!.camera!!.isFlatCameraModeSupported)
                return@get call.respond(CommandCompleted(false, "Only Flat-Camera Mode is supported"))

            val stopVideoError = suspendCoroutine<DJIError?> { cont ->
                drone!!.camera!!.stopRecordVideo { error ->
                    cont.resume(error)
                }
            }

            if(stopVideoError != null)
                return@get call.respond(CommandCompleted(false, "Error in stopping video recording" +  stopVideoError.description))

            return@get call.respond(CommandCompleted(true, null))
        }

        get("/pitchGimbal/{angle}") {

            // Parse Angle
            val angle = try {
                -call.parameters["angle"]!!.toFloat()
            } catch (e: Exception) {
                0f
            }

            if(angle > 0 || angle < -90)
                return@get call.respond(CommandCompleted(false, "Pitch Angle Must be between 0 and 90 deg"))

            showToast("Made it here!")
            // Availability Checks
            if(drone == null)
                return@get call.respond(CommandCompleted(false, "Drone not Available"))
            if(drone!!.gimbal == null)
                return@get call.respond(CommandCompleted(false, "Gimbal not Available"))

            val gimbalModeError = suspendCoroutine<DJIError?> { cont ->
                drone!!.gimbal!!.setMode(GimbalMode.FPV) { error ->
                    cont.resume(error)
                }
            }

            if(gimbalModeError != null)
                return@get call.respond(CommandCompleted(false, "Gimbal Mode Error: " + gimbalModeError.description))


            val gimbalRotation = Rotation.Builder().mode(RotationMode.ABSOLUTE_ANGLE).pitch(angle).yaw(Rotation.NO_ROTATION).roll(Rotation.NO_ROTATION).time(1.0).build()


            val gimbalRotationError = suspendCoroutine<DJIError?> { cont ->
                drone!!.gimbal!!.rotate(gimbalRotation) { error ->
                    cont.resume(error)
                }
            }

            if(gimbalRotationError != null)
                return@get call.respond(CommandCompleted(false, "Gimbal Rotation Error: " + gimbalRotationError.description))

            return@get call.respond(CommandCompleted(true, null))
        }

        get("/capturePanorama") {

            // Availability Checks
            if(drone == null)
                return@get call.respond(CommandCompleted(false, "Drone not Available"))
            if(drone!!.camera == null)
                return@get call.respond(CommandCompleted(false, "Camera not Available"))
            if(drone!!.camera!!.mediaManager == null)
                return@get call.respond(CommandCompleted(false, "Media Manager Not Available"))
            if(!drone!!.camera!!.isFlatCameraModeSupported)
                return@get call.respond(CommandCompleted(false, "Only Flat-Camera Mode is supported"))

            // Get out of playback mode if in it
            suspendCoroutine<DJIError?> { cont ->
                drone!!.camera!!.exitPlayback { error ->
                    cont.resume(error)
                }
            }

            val flatModeError = suspendCoroutine<DJIError?> { cont ->
                drone!!.camera!!.setFlatMode(SettingsDefinitions.FlatCameraMode.PHOTO_PANORAMA) { error ->
                    cont.resume(error)
                }
            }

            if(flatModeError != null)
                return@get call.respond(CommandCompleted(false, "Error in setting flat mode: " + flatModeError.description))

            val panoramaModeError = suspendCoroutine<DJIError?> { cont ->
                drone!!.camera!!.setPhotoPanoramaMode(SettingsDefinitions.PhotoPanoramaMode.PANORAMA_MODE_3X1) { error ->
                    cont.resume(error)
                }
            }

            if(panoramaModeError != null)
                return@get call.respond(CommandCompleted(false, "Error in setting Panorama mode: " + panoramaModeError.description))

            val panoramaShotError = suspendCoroutine<DJIError?> { cont ->
                drone!!.camera!!.startShootPhoto { error ->
                    cont.resume(error)
                }
            }

            if(panoramaShotError != null)
                return@get call.respond(CommandCompleted(false, "Error in taking panorama shot: " + panoramaShotError.description))

            return@get call.respond(CommandCompleted(true, null))

        }


        get("/fetchPreviewFromIndex/{n}") {

            // Parse Index
            val n = try {
                val parsedIdx = call.parameters["n"]!!.toInt()
                assert(parsedIdx >= 0)
                parsedIdx
            } catch (e: Exception) {
                0
            }

            // Availability Checks
            if(drone == null)
                return@get call.respond(CommandCompleted(false, "Drone not Available"))
            if(drone!!.camera == null)
                return@get call.respond(CommandCompleted(false, "Camera not Available"))
            if(drone!!.camera!!.mediaManager == null)
                return@get call.respond(CommandCompleted(false, "Media Manager Not Available"))
            if(!drone!!.camera!!.isFlatCameraModeSupported)
                return@get call.respond(CommandCompleted(false, "Only Flat-Camera Mode is supported"))

            var exitPlaybackError = suspendCoroutine<DJIError?> { cont ->
                drone!!.camera!!.exitPlayback { error ->
                    cont.resume(error)
                }
            }

            if(exitPlaybackError != null)
                return@get call.respond(CommandCompleted(false, "Error in initial Playback mode exit: " + exitPlaybackError.description))

            // Refresh File List
            val refreshError = suspendCoroutine<DJIError?> { cont ->
                drone!!.camera!!.mediaManager!!.refreshFileListOfStorageLocation(SettingsDefinitions.StorageLocation.SDCARD) { error ->
                    cont.resume(error)
                }
            }

            if(refreshError != null)
                return@get call.respond(CommandCompleted(false, "Error in Refreshing File List: " + refreshError.description))

            // Fetch Media Files and Sort
            val mediaFiles = drone!!.camera!!.mediaManager!!.sdCardFileListSnapshot
                ?: return@get call.respond(CommandCompleted(false, "Error in fetching File List Snapshot"))

            if (n >= mediaFiles.size)
                return@get call.respond(CommandCompleted(false, "Index $n greater than number of images available (${mediaFiles.size})"))

            mediaFiles.sortByDescending { it.timeCreated }

            // Get target file
            val targetFile = mediaFiles[n]

            // Fetch target file preview
            if(targetFile.preview == null)
            {
                val previewFetchError = suspendCoroutine<DJIError?> { cont ->
                    targetFile.fetchPreview { error ->
                        cont.resume(error)
                    }
                }

                if(previewFetchError != null)
                    return@get call.respond(CommandCompleted(false, "Error in fetching preview: " + previewFetchError.description + " Target File Name: " + targetFile.fileName))
            }

            // Convert image to base64 string
            val previewString = bitmapToString(targetFile.preview)

            // Exit Playback Mode
            exitPlaybackError = suspendCoroutine { cont ->
                drone!!.camera!!.exitPlayback { error ->
                    cont.resume(error)
                }
            }

            if(exitPlaybackError != null)
                return@get call.respond(CommandCompleted(false, "Error in exiting playback mode: " + exitPlaybackError.description))

            // Return Preview String
            return@get call.respond(DroneState(previewString))

        }

    }

    private fun Route.takeoffAndLandControl() {

        get("/takeoff") {
                if(drone != null) {
                    val takeoffError = suspendCoroutine<DJIError?> { cont ->
                        drone?.flightController?.startTakeoff { takeoffError ->
                            cont.resume(takeoffError)
                        }
                    }
                    call.respond(CommandCompleted(takeoffError == null, takeoffError?.description))
                }
            else {
                call.respond(CommandCompleted(false, "Drone Not Available"))
            }
        }

        get("/land") {
            if(drone != null){
                val descentError = suspendCoroutine<DJIError?> { cont ->
                    drone?.flightController?.startLanding { descendError ->
                        cont.resume(descendError)
                    }
                }
                call.respond(CommandCompleted(descentError == null, descentError?.description))
            }
            else {
                call.respond(CommandCompleted(false, "Drone Not Available"))
            }
        }

        get("/confirmLanding") {
            if (drone != null) {
                val landingError = suspendCoroutine<DJIError?> { cont ->
                    drone?.flightController?.confirmLanding { landingError ->
                        cont.resume(landingError)
                    }
                }
                call.respond(CommandCompleted(landingError == null, landingError?.description))
            } else {
                call.respond(CommandCompleted(false, "Drone not Available"))
            }
        }

    }

    private fun Route.livestreamControl() {

        get("/livestream/start/{url}") {
            val streamUrl = call.parameters["url"]
            val livestreamManager = dji.sdk.sdkmanager.DJISDKManager.getInstance().liveStreamManager

            if (livestreamManager == null) {
                call.respond(CommandCompleted(false, "Drone not connected or livestream not supported (code: -1)"))
                return@get
            }

            if (streamUrl.isNullOrBlank()) {
                call.respond(CommandCompleted(false, "Missing RTMP URL (code: -2)"))
                return@get
            }

            livestreamManager.setLiveUrl(streamUrl)

            // Set video encode parameters BEFORE start()
            livestreamManager.setLiveVideoBitRateMode(LiveVideoBitRateMode.AUTO)
//            livestreamManager.setLiveVideoBitRate(512.0F) // kbps, adjust as needed

// If supported by your SDK/drone:
            livestreamManager.setLiveVideoResolution(LiveVideoResolution.VIDEO_RESOLUTION_480_360)


            val started = livestreamManager.startStream()
            val res = livestreamManager.getLiveVideoResolution()
            val bitrate = livestreamManager.getLiveVideoBitRate()
            call.respond(CommandCompleted(true, "Livestream start reslt: code - $started, res - $res, bitrate - $bitrate"))
        }

        get("/livestream/stop") {
            val livestreamManager = dji.sdk.sdkmanager.DJISDKManager.getInstance().liveStreamManager

            if (livestreamManager == null) {
                call.respond(CommandCompleted(false, "Drone not connected or livestream not supported (code: -1)"))
                return@get
            }

            livestreamManager.stopStream()
            call.respond(CommandCompleted(true, "Livestream stopped async. Run isStreaming to check real time status."))

        }
    }

    private fun Route.velocityControl() {

        get("/startVelocityControl") {
            if(drone != null) {
                try {
                    if(controlMode == ControlMode.POSITION)
                        throw DJIControlException("Cannot use VELOCITY command in POSITION control mode")

                    setVirtualSticks(true)
                    setCurrentControlModes(Triple(VerticalControlMode.VELOCITY, YawControlMode.ANGULAR_VELOCITY, RollPitchControlMode.VELOCITY))

                    val handler = Handler(Looper.getMainLooper())

                    followingVelocityCommands = true

                    velocityControlRunnable = object: Runnable {
                        override fun run() {
                            drone?.flightController?.sendVirtualStickFlightControlData(FlightControlData(velocityModeYVel, velocityModeXVel, velocityModeYawVel, velocityModeZVel)) {
                                if(followingVelocityCommands)
                                    handler.postDelayed(this, flightCommandInterval)
                            }
                        }
                    }

                    handler.postDelayed(velocityControlRunnable as Runnable, flightCommandInterval)

                    call.respond(CommandCompleted(true, null))
                }
                catch(e: DJIControlException) {
                    call.respond(CommandCompleted(false, e.message))
                }
                catch(e:Exception) {
                    call.respond(CommandCompleted(false, e.message))
                }
            }
            else
                call.respond(CommandCompleted(false, "Drone Not Available"))

        }

        get("/setVelocityCommand/{xVel}/{yVel}/{zVel}/{yawVel}") {
            if(drone != null) {
                try {
                    if(controlMode == ControlMode.POSITION)
                        throw DJIControlException("Cannot use VELOCITY command in POSITION control mode")
                    if(!followingVelocityCommands)
                        throw DJIControlException("Cannot set velocity commands before starting Velocity Control")
                    val xVel = call.parameters["xVel"]!!.toFloat()
                    val yVel = call.parameters["yVel"]!!.toFloat()
                    val zVel = call.parameters["zVel"]!!.toFloat()
                    val yawVel = call.parameters["yawVel"]!!.toFloat()

                    velocityModeXVel = xVel
                    velocityModeYVel = yVel
                    velocityModeZVel = zVel
                    velocityModeYawVel = yawVel

                    call.respond(CommandCompleted(true, null))
                }
                catch(e: NumberFormatException) {
                    call.respond(CommandCompleted(false, "Velocities must be valid floats."))
                }
                catch(e: DJIControlException) {
                    call.respond(CommandCompleted(false, e.message))
                }
                catch(e: Exception) {
                    call.respond(CommandCompleted(false, e.message))
                }
            }
            else
                call.respond(CommandCompleted(false, "Drone Not Available"))
        }

        get("/getCurrentVelocityCommand") {
            if(drone != null) {
                try {
                    if(controlMode == ControlMode.POSITION)
                        throw DJIControlException("Cannot use VELOCITY command in POSITION control mode")

                    call.respond(VelocityCommand(velocityModeXVel, velocityModeYVel, velocityModeZVel, velocityModeYawVel))
                }
                catch(e: DJIControlException) {
                    call.respond(CommandCompleted(false, e.message))
                }
                catch(e: Exception) {
                    call.respond(CommandCompleted(false, e.message))
                }
            }
            else
                call.respond(CommandCompleted(false, "Drone Not Available"))
        }

        get("/stopVelocityControl") {
            if(drone != null) {
                try {
                    if(controlMode == ControlMode.POSITION)
                        throw DJIControlException("Cannot use VELOCITY command in POSITION control mode")
                    setVirtualSticks(false)
                    followingVelocityCommands = false
                    velocityModeXVel = 0f
                    velocityModeYawVel = 0f
                    velocityModeZVel = 0f
                    velocityModeYawVel = 0f
                    val handler = Handler(Looper.getMainLooper())
                    handler.removeCallbacks(velocityControlRunnable as Runnable)
                    call.respond(CommandCompleted(true, null))
                }
                catch(e: DJIControlException) {
                    call.respond(CommandCompleted(false, e.message))
                }
                catch(e: Exception) {
                    call.respond(CommandCompleted(false, e.message))
                }
            }
            else
                call.respond(CommandCompleted(false, "Drone Not Available"))
        }

    }

    private fun Route.throttleControl() {

        get("/moveUp/{dist}") {
            if(drone != null) {
                if(call.parameters["dist"] != null) {
                    val currentControlModes = getCurrentControlModes()
                    try {
                        if(controlMode == ControlMode.VELOCITY)
                            throw DJIControlException("Cannot use POSITION command in VELOCITY control mode")
                        val dist = getNumVal(call.parameters["dist"].toString())
                        setVirtualSticks(true)
                        setCurrentControlModes(Triple(VerticalControlMode.VELOCITY, YawControlMode.ANGULAR_VELOCITY, RollPitchControlMode.VELOCITY))

                        createMotionPlanAndExecute(dist, Direction.UP)

                        setVirtualSticks(false)
                        call.respond(CommandCompleted(true, null))
                    }
                    catch (e: NumberFormatException) {
                        call.respond(CommandCompleted(false, e.message))
                    }
                    catch(e: DJIControlException) {
                        call.respond(CommandCompleted(false, e.message))
                    }
                    finally {
                        setCurrentControlModes(currentControlModes)
                    }

                }
                else
                    call.respond(CommandCompleted(false, "Missing distance value"))
            }
            else
                call.respond(CommandCompleted(false, "Drone not Available"))

        }

        get("/moveDown/{dist}") {
            if(drone != null) {
                if(call.parameters["dist"] != null) {
                    val currentControlModes = getCurrentControlModes()
                    try {
                        if(controlMode == ControlMode.VELOCITY)
                            throw DJIControlException("Cannot use POSITION command in VELOCITY control mode")
                        val dist = getNumVal(call.parameters["dist"].toString())
                        setVirtualSticks(true)
                        setCurrentControlModes(Triple(VerticalControlMode.VELOCITY, YawControlMode.ANGULAR_VELOCITY, RollPitchControlMode.VELOCITY))

                        createMotionPlanAndExecute(dist, Direction.DOWN)

                        setVirtualSticks(false)
                        call.respond(CommandCompleted(true, null))
                    }
                    catch (e: NumberFormatException) {
                        call.respond(CommandCompleted(false, e.message))
                    }
                    catch(e: DJIControlException) {
                        call.respond(CommandCompleted(false, e.message))
                    }
                    finally {
                        setCurrentControlModes(currentControlModes)
                    }

                }
                else
                    call.respond(CommandCompleted(false, "Missing distance value"))
            }
            else
                call.respond(CommandCompleted(false, "Drone not Available"))

        }

    }

    private fun Route.yawControl() {
        get("/rotateClockwise/{angle}") {
            if(drone != null) {
                if(call.parameters["angle"] != null) {
                    val currentControlModes = getCurrentControlModes()
                    try {
                        if(controlMode == ControlMode.VELOCITY)
                            throw DJIControlException("Cannot use POSITION command in VELOCITY control mode")
                        val angle = getNumVal(call.parameters["angle"].toString())
                        setVirtualSticks(true)
                        setCurrentControlModes(Triple(VerticalControlMode.VELOCITY, YawControlMode.ANGULAR_VELOCITY, RollPitchControlMode.VELOCITY))

                        createMotionPlanAndExecute(angle, Direction.CLOCKWISE)

                        setVirtualSticks(false)
                        call.respond(CommandCompleted(true, null))
                    }
                    catch (e: NumberFormatException) {
                        call.respond(CommandCompleted(false, e.message))
                    }
                    catch(e: DJIControlException) {
                        call.respond(CommandCompleted(false, e.message))
                    }
                    finally {
                        setCurrentControlModes(currentControlModes)
                    }

                }
                else
                    call.respond(CommandCompleted(false, "Missing distance value"))
            }
            else
                call.respond(CommandCompleted(false, "Drone not Available"))
        }

        get("/rotateCounterClockwise/{angle}") {
            if(drone != null) {
                if(call.parameters["angle"] != null) {
                    val currentControlModes = getCurrentControlModes()
                    try {
                        if(controlMode == ControlMode.VELOCITY)
                            throw DJIControlException("Cannot use POSITION command in VELOCITY control mode")
                        val angle = getNumVal(call.parameters["angle"].toString())
                        setVirtualSticks(true)
                        setCurrentControlModes(Triple(VerticalControlMode.VELOCITY, YawControlMode.ANGULAR_VELOCITY, RollPitchControlMode.VELOCITY))

                        createMotionPlanAndExecute(angle, Direction.COUNTER_CLOCKWISE)

                        setVirtualSticks(false)
                        call.respond(CommandCompleted(true, null))
                    }
                    catch (e: NumberFormatException) {
                        call.respond(CommandCompleted(false, e.message))
                    }
                    catch(e: DJIControlException) {
                        call.respond(CommandCompleted(false, e.message))
                    }
                    finally {
                        setCurrentControlModes(currentControlModes)
                    }

                }
                else
                    call.respond(CommandCompleted(false, "Missing distance value"))
            }
            else
                call.respond(CommandCompleted(false, "Drone not Available"))
        }
    }

    private fun Route.rollPitchControl() {
        get("/moveForward/{dist}") {
            if(drone != null) {
                if(call.parameters["dist"] != null) {
                    val currentControlModes = getCurrentControlModes()
                    try {
                        if(controlMode == ControlMode.VELOCITY)
                            throw DJIControlException("Cannot use POSITION command in VELOCITY control mode")
                        val dist = getNumVal(call.parameters["dist"].toString())
                        setVirtualSticks(true)
                        setCurrentControlModes(Triple(VerticalControlMode.VELOCITY, YawControlMode.ANGULAR_VELOCITY, RollPitchControlMode.VELOCITY))

                       createMotionPlanAndExecute(dist, Direction.FORWARD)

                        setVirtualSticks(false)
                        call.respond(CommandCompleted(true, null))
                    }
                    catch (e: NumberFormatException) {
                        call.respond(CommandCompleted(false, e.message))
                    }
                    catch(e: DJIControlException) {
                        call.respond(CommandCompleted(false, e.message))
                    }
                    finally {
                        setCurrentControlModes(currentControlModes)
                    }

                }
                else
                    call.respond(CommandCompleted(false, "Missing distance value"))
            }
            else
                call.respond(CommandCompleted(false, "Drone not Available"))
        }

        get("/moveBackward/{dist}") {
            if(drone != null) {
                if(call.parameters["dist"] != null) {
                    val currentControlModes = getCurrentControlModes()
                    try {
                        if(controlMode == ControlMode.VELOCITY)
                            throw DJIControlException("Cannot use POSITION command in VELOCITY control mode")
                        val dist = getNumVal(call.parameters["dist"].toString())
                        setVirtualSticks(true)
                        setCurrentControlModes(Triple(VerticalControlMode.VELOCITY, YawControlMode.ANGULAR_VELOCITY, RollPitchControlMode.VELOCITY))

                        createMotionPlanAndExecute(dist, Direction.BACKWARD)

                        setVirtualSticks(false)
                        call.respond(CommandCompleted(true, null))
                    }
                    catch (e: NumberFormatException) {
                        call.respond(CommandCompleted(false, e.message))
                    }
                    catch(e: DJIControlException) {
                        call.respond(CommandCompleted(false, e.message))
                    }
                    finally {
                        setCurrentControlModes(currentControlModes)
                    }

                }
                else
                    call.respond(CommandCompleted(false, "Missing distance value"))
            }
            else
                call.respond(CommandCompleted(false, "Drone not Available"))
        }

        get("/moveRight/{dist}") {
            if(drone != null) {
                if(call.parameters["dist"] != null) {
                    val currentControlModes = getCurrentControlModes()
                    try {
                        if(controlMode == ControlMode.VELOCITY)
                            throw DJIControlException("Cannot use POSITION command in VELOCITY control mode")
                        val dist = getNumVal(call.parameters["dist"].toString())
                        setVirtualSticks(true)
                        setCurrentControlModes(Triple(VerticalControlMode.VELOCITY, YawControlMode.ANGULAR_VELOCITY, RollPitchControlMode.VELOCITY))

                        createMotionPlanAndExecute(dist, Direction.RIGHT)

                        setVirtualSticks(false)
                        call.respond(CommandCompleted(true, null))
                    }
                    catch (e: NumberFormatException) {
                        call.respond(CommandCompleted(false, e.message))
                    }
                    catch(e: DJIControlException) {
                        call.respond(CommandCompleted(false, e.message))
                    }
                    finally {
                        setCurrentControlModes(currentControlModes)
                    }

                }
                else
                    call.respond(CommandCompleted(false, "Missing distance value"))
            }
            else
                call.respond(CommandCompleted(false, "Drone not Available"))
        }

        get("/moveLeft/{dist}") {
            if(drone != null) {
                if(call.parameters["dist"] != null) {
                    val currentControlModes = getCurrentControlModes()
                    try {
                        if(controlMode == ControlMode.VELOCITY)
                            throw DJIControlException("Cannot use POSITION command in VELOCITY control mode")
                        val dist = getNumVal(call.parameters["dist"].toString())
                        setVirtualSticks(true)
                        setCurrentControlModes(Triple(VerticalControlMode.VELOCITY, YawControlMode.ANGULAR_VELOCITY, RollPitchControlMode.VELOCITY))

                        createMotionPlanAndExecute(dist, Direction.LEFT)

                        setVirtualSticks(false)
                        call.respond(CommandCompleted(true, null))
                    }
                    catch (e: NumberFormatException) {
                        call.respond(CommandCompleted(false, e.message))
                    }
                    catch(e: DJIControlException) {
                        call.respond(CommandCompleted(false, e.message))
                    }
                    finally {
                        setCurrentControlModes(currentControlModes)
                    }

                }
                else
                    call.respond(CommandCompleted(false, "Missing distance value"))
            }
            else
                call.respond(CommandCompleted(false, "Drone not Available"))
        }
    }

    // Motion planning functions
    private suspend fun createMotionPlanAndExecute(dist: Float, direction: Direction){
        val commands = when (velocityProfile) {
            VelocityProfile.CONSTANT -> generateConstantVelocityProfile(dist, direction)
            VelocityProfile.TRAPEZOIDAL -> generateTrapezoidalVelocityProfile(dist, direction)
            VelocityProfile.S_CURVE -> generateSCurveVelocityProfile(dist, direction)
        }
        for(command in commands) {
            drone?.flightController?.sendVirtualStickFlightControlData(command, null)
            delay(flightCommandInterval)
        }
    }

    private fun generateVelocitiesFromFunction(v: (Float)->Float, tTotal: Float, direction: Direction): MutableList<FlightControlData> {
        val tSteps = arange(0f, tTotal, flightCommandInterval/1000f)

        val velocities = mutableListOf<FlightControlData>()

        for(t in tSteps) {
            val command = when(direction) {
                Direction.FORWARD -> FlightControlData(0f, v(t), 0f, 0f)
                Direction.BACKWARD -> FlightControlData(0f, -v(t), 0f, 0f)
                Direction.RIGHT -> FlightControlData(v(t), 0f, 0f, 0f)
                Direction.LEFT -> FlightControlData(-v(t), 0f, 0f, 0f)
                Direction.UP -> FlightControlData(0f, 0f, 0f, v(t))
                Direction.DOWN -> FlightControlData(0f, 0f, 0f, -v(t))
                Direction.CLOCKWISE -> FlightControlData(0f, 0f, v(t), 0f)
                Direction.COUNTER_CLOCKWISE -> FlightControlData(0f, 0f, -v(t), 0f)
            }
            velocities.add(command)
        }
        velocities.add(FlightControlData(0f, 0f, 0f, 0f))
        return velocities
    }

    private fun generateConstantVelocityProfile(D: Float, direction: Direction) : MutableList<FlightControlData>{
        val v = if (direction == Direction.CLOCKWISE || direction == Direction.COUNTER_CLOCKWISE) maxAngularSpeed else maxSpeed
        val tTotal = D/v

        val vFn = fun(_:Float): Float {
            return v
        }

        return generateVelocitiesFromFunction(vFn, tTotal, direction)
    }

    private fun generateTrapezoidalVelocityProfile(D: Float, direction: Direction): MutableList<FlightControlData> {

        // Constraints
        val a = if(direction == Direction.CLOCKWISE || direction == Direction.COUNTER_CLOCKWISE) maxAngularAcceleration else maxAcceleration
        val vMax = min(D/2, if(direction == Direction.CLOCKWISE || direction == Direction.COUNTER_CLOCKWISE) maxAngularSpeed else maxSpeed )

        // Time taken at each phase
        val tp1 = vMax/a
        val tp2 = (a*D - (vMax.pow(2)))/(a*vMax)
        val tp3 = vMax/a

        // Total time taken
        val tTotal = tp1 + tp2 + tp3

        val vFn = fun(t:Float):Float {
            return if(t <= tp1)
                a*t
            else if(t <= tp1 + tp2)
                vMax
            else
                vMax - a*(t - tp1 - tp2)
        }

        return generateVelocitiesFromFunction(vFn, tTotal, direction)
    }

    private fun generateSCurveVelocityProfile(D:Float, direction: Direction): MutableList<FlightControlData> {

        // Constraints
        val j = if(direction == Direction.CLOCKWISE || direction == Direction.COUNTER_CLOCKWISE) maxAngularJerk else maxJerk
        val vMax = min(D/2, if(direction == Direction.CLOCKWISE || direction == Direction.COUNTER_CLOCKWISE) maxAngularSpeed else maxSpeed )
        val aMax = min(0.75f*vMax, if(direction == Direction.CLOCKWISE || direction == Direction.COUNTER_CLOCKWISE) maxAngularAcceleration else maxAcceleration )

        // Final velocity for each phase
        val vf1 = aMax.pow(2)/(2*j)
        val vf2 = vMax - aMax.pow(2)/(2*j)
        val vf3 = vMax
        val vf4 = vMax
        val vf5 = vMax - aMax.pow(2)/(2*j)
        val vf6 = aMax.pow(2)/(2*j)

        // Time taken for each phase
        val tp1 = aMax/j
        val tp2 = (vf2 - vf1)/aMax
        val tp3 = aMax/j
        val tp4 = ((aMax * j * D) - (vMax * aMax.pow(2)) - (j*vMax.pow(2)))/(j*aMax*vMax)
        val tp5 = aMax/j
        val tp6 = (vf5- vf6)/aMax
        val tp7 = aMax/j

        // Total time taken
        val tTotal = tp1 + tp2 + tp3 + tp4 + tp5 + tp6 + tp7

        val vFn = fun (t: Float): Float {
            if(t <= tp1)
            {
                return j * t.pow(2) / 2
            }
            else if(t <= tp1 + tp2){
                val tCurr = t - tp1
                return vf1 + (aMax * tCurr)
            }
            else if(t <= tp1 + tp2 + tp3) {
                val tCurr = t - tp1 - tp2
                return vf2 + aMax*tCurr - (j*tCurr.pow(2))/2
            }
            else if(t <= tp1 + tp2 + tp3 + tp4) {
                return vf3
            }
            else if(t <= tp1 + tp2 + tp3 + tp4 + tp5) {
                val tCurr = t - tp1 - tp2 - tp3 - tp4
                return vf4 - (j*tCurr.pow(2))/2
            }
            else if(t <= tp1 + tp2 + tp3 + tp4 + tp5 + tp6) {
                val tCurr = t - tp1 - tp2 - tp3 - tp4 - tp5
                return vf5 - aMax*tCurr
            }
            else{
                val tCurr = t - tp1 - tp2 - tp3 - tp4 - tp5 - tp6
                return vf6 - aMax*tCurr + (j*tCurr.pow(2))/2
            }
        }
        return generateVelocitiesFromFunction(vFn, tTotal, direction)
    }

    // Numeric Utility
    private fun getNumVal(numStr: String): Float {
        val num = numStr.toFloat()
        if (num <= 0)
            throw NumberFormatException("Non-Positive Float not allowed")
        return num
    }

    private fun arange(start:Float, stop:Float, step:Float) : MutableList<Float> {
        val a = mutableListOf<Float>()
        var i = start
        while(i < stop) {
            a.add(i)
            i += step
        }
        return a
    }

    // Misc Utility
    private fun bitmapToString(bitmap: Bitmap): String {
        val byteStream = ByteArrayOutputStream()
        bitmap.compress(Bitmap.CompressFormat.PNG, 100, byteStream)
        val bytes = byteStream.toByteArray()
        return Base64.encodeToString(bytes, Base64.NO_WRAP)
    }

    // Flight controller state utility
    private suspend fun setVirtualSticks(enable: Boolean) {
        val virtualSticksError = suspendCoroutine<DJIError?> { cont ->
            drone?.flightController?.setVirtualStickModeEnabled(enable) {
                cont.resume(it)
            }
        }
        val action = if (enable) "Enable" else "Disable"

        if(virtualSticksError != null)
            throw DJIControlException("Cannot $action Virtual Sicks")
    }

    private fun getCurrentControlModes(): Triple<VerticalControlMode?, YawControlMode?, RollPitchControlMode?> {
        val vcm = drone?.flightController?.verticalControlMode
        val ycm = drone?.flightController?.yawControlMode
        val rpcm = drone?.flightController?.rollPitchControlMode

        return Triple(vcm, ycm, rpcm)
    }

    private fun setCurrentControlModes(desiredControlModes: Triple<VerticalControlMode?, YawControlMode?, RollPitchControlMode?>) {
        drone?.flightController?.verticalControlMode = desiredControlModes.first
        drone?.flightController?.yawControlMode = desiredControlModes.second
        drone?.flightController?.rollPitchControlMode = desiredControlModes.third
    }

    // UI Handling Functions
    private fun updateDroneDetails() {

        drone?.also { drone ->

            if(drone.isConnected) {
                drone.getName(object: CommonCallbacks.CompletionCallbackWith<String> {
                    override fun onSuccess(droneName: String?) {
                        droneNameText.text = droneName
                    }

                    override fun onFailure(error: DJIError?) {
                        droneNameText.text = getString(R.string.unknown_drone)
                        if(error != null)
                            showToast(error.description)
                    }
                })

                drone.battery.also { battery ->
                  battery.setStateCallback {
                      batteryText.text = getString(R.string.battery_percentage, it.chargeRemainingInPercent, "%")
                  }
                }
            }
            else {
                droneNameText.text = getString(R.string.drone_disconnected)
                batteryText.text = ""
                removeComponentCallbacks()
            }
        }?: run {
            droneNameText.text = getString(R.string.none_connected)
            batteryText.text = ""
        }
    }

    private fun setupVideoPipelineIfReady() {
        // Only setup if textureView is available and surface is ready
        if (::textureView.isInitialized && textureView.isAvailable) {
            codecManager = DJICodecManager(this, textureView.surfaceTexture, textureView.width, textureView.height)
            VideoFeeder.getInstance().primaryVideoFeed.addVideoDataListener(videoDataListener)
            Log.d(TAG, "DJI Video pipeline initialized")
        } else {
            // Wait until surface is available
            textureView.surfaceTextureListener = object : TextureView.SurfaceTextureListener {
                override fun onSurfaceTextureAvailable(surface: SurfaceTexture, width: Int, height: Int) {
                    codecManager = DJICodecManager(this@MainActivity, surface, width, height)
                    VideoFeeder.getInstance().primaryVideoFeed.addVideoDataListener(videoDataListener)
                    Log.d(TAG, "DJI Video pipeline initialized (late)")
                }
                override fun onSurfaceTextureSizeChanged(surface: SurfaceTexture, width: Int, height: Int) {}
                override fun onSurfaceTextureDestroyed(surface: SurfaceTexture): Boolean = false
                override fun onSurfaceTextureUpdated(surface: SurfaceTexture) {}
            }
        }
    }

    private fun removeComponentCallbacks() {
        drone?.battery?.setStateCallback(null)
    }

    // SDK Callback Functions
    override fun onRegister(error: DJIError?) {
        if(error == DJISDKError.REGISTRATION_SUCCESS)
            regText.text = getString(R.string.registered, sdkManager.sdkVersion)
        else{
            regText.text = getString(R.string.register_failed)
            Log.i(TAG, "onRegister Failed: ${error?.description}")
        }
    }

    override fun onProductConnect(product: BaseProduct?) {
        drone = product as Aircraft?

        drone?.flightController?.rollPitchCoordinateSystem = FlightCoordinateSystem.BODY

        updateDroneDetails()
        notifyStatusChanged()
        // Now set up video pipeline since SDK and product are ready!
        setupVideoPipelineIfReady()

    }

    override fun onProductChanged(product: BaseProduct?) {
        drone = product as Aircraft?

        drone?.flightController?.rollPitchCoordinateSystem = FlightCoordinateSystem.BODY

        updateDroneDetails()
        notifyStatusChanged()
    }

    override fun onProductDisconnect() {
        removeComponentCallbacks()
        drone = null
        updateDroneDetails()
        notifyStatusChanged()
    }

    override fun onDestroy() {
        super.onDestroy()
        VideoFeeder.getInstance().primaryVideoFeed.removeVideoDataListener(videoDataListener)
        codecManager?.cleanSurface()
        codecManager = null
//        stopPixelCopyProducerLoop()    // <-- add this
//        stopInstrumentation()
    }

    override fun onComponentChange(
        p0: BaseProduct.ComponentKey?,
        p1: BaseComponent?,
        p2: BaseComponent?
    ) {}

    override fun onInitProcess(p0: DJISDKInitEvent?, p1: Int) {}

    override fun onDatabaseDownloadProgress(p0: Long, p1: Long) {}

    // UI Utility Functions
    private fun notifyStatusChanged() {
        mHandler.removeCallbacks(updateRunnable)
        mHandler.postDelayed(updateRunnable, 500)
    }

    private val updateRunnable = Runnable {
        val intent = Intent(FLAG_CONNECTION_CHANGE)
        sendBroadcast(intent)
    }

    private fun showToast(text: String) {
        val handler = Handler(Looper.getMainLooper())
        handler.post {
            Toast.makeText(this, text, Toast.LENGTH_LONG).show()
        }
    }
}

//class MjpegStreamServer(port: Int = 8090) : NanoHTTPD(port) {
//    private val frameQueue = LinkedBlockingQueue<ByteArray>(6) // a slightly larger buffer
//    // keep older method for compatibility if you want
//    fun pushFrame(jpeg: ByteArray) {
//        // simple wrapper to preserve older callers; delegates to offerFrame
//        offerFrame(jpeg)
//    }
//
//    // New: safer offer that returns whether enqueue succeeded and logs dropped frames
//    fun offerFrame(jpeg: ByteArray): Boolean {
//        try {
//            if (frameQueue.offer(jpeg)) return true
//            // queue is full: drop the oldest and try again
//            val dropped = frameQueue.poll()
//            val ok = frameQueue.offer(jpeg)
//            if (ok) {
//                android.util.Log.w("MjpegStreamServer", "offerFrame: queue full - dropped oldest frame size=${dropped?.size}")
//            } else {
//                // extremely unlikely, but log if it fails again
//                android.util.Log.e("MjpegStreamServer", "offerFrame: failed to enqueue even after dropping one frame")
//            }
//            return ok
//        } catch (e: Exception) {
//            android.util.Log.e("MjpegStreamServer", "offerFrame: exception while offering frame", e)
//            return false
//        }
//    }
//
//    // New: expose current queue size for diagnostics
//    fun queueSize(): Int = frameQueue.size
//
//    override fun serve(session: IHTTPSession): Response {
//        val boundary = "boundary"
//        val response = newChunkedResponse(
//            Response.Status.OK,
//            "multipart/x-mixed-replace; boundary=--$boundary",
//            MJpegInputStream(boundary, frameQueue)
//        )
//        response.addHeader("Connection", "close")
//        response.addHeader("Cache-Control", "no-cache")
//        return response
//    }
//
//
//    class MJpegInputStream(
//        private val boundary: String,
//        private val frameQueue: LinkedBlockingQueue<ByteArray>
//    ) : java.io.InputStream() {
//        private var currentFrame: ByteArray? = null
//        private var currentIndex = 0
//
//        // Poll timeout waiting for frames (ms)
//        private val FRAME_POLL_TIMEOUT_MS = 2000L
//        // If we poll this many times without a frame, end the stream.
//        private val MAX_EMPTY_POLLS = 3
//        private var consecutiveEmptyPolls = 0
//
//        override fun read(): Int {
//            try {
//                if (currentFrame == null || currentIndex >= currentFrame!!.size) {
//                    // Wait up to FRAME_POLL_TIMEOUT_MS for next frame
//                    val frame = frameQueue.poll(FRAME_POLL_TIMEOUT_MS, java.util.concurrent.TimeUnit.MILLISECONDS)
//                    if (frame == null) {
//                        consecutiveEmptyPolls++
//                        android.util.Log.w("MJpegInputStream", "No frame available (timeout #$consecutiveEmptyPolls)")
//                        if (consecutiveEmptyPolls >= MAX_EMPTY_POLLS) {
//                            android.util.Log.i("MJpegInputStream", "Max empty polls reached — signalling EOF to close client")
//                            return -1 // EOF -> server closes connection
//                        }
//                        // No frame this cycle — wait again next read
//                        // Small sleep to avoid tight loop if caller keeps calling read rapidly
//                        Thread.sleep(50)
//                        return read()
//                    } else {
//                        consecutiveEmptyPolls = 0
//                        val header = ("\r\n--$boundary\r\nContent-Type: image/jpeg\r\nContent-Length: ${frame.size}\r\n\r\n").toByteArray()
//                        currentFrame = ByteArray(header.size + frame.size)
//                        System.arraycopy(header, 0, currentFrame, 0, header.size)
//                        System.arraycopy(frame, 0, currentFrame, header.size, frame.size)
//                        currentIndex = 0
//                        android.util.Log.d("MJpegInputStream", "Pushing frame to client: ${frame.size} bytes (total ${currentFrame!!.size})")
//                    }
//                }
//                return currentFrame!![currentIndex++].toInt() and 0xFF
//            } catch (ie: InterruptedException) {
//                Thread.currentThread().interrupt()
//                return -1
//            } catch (ex: Exception) {
//                android.util.Log.e("MJpegInputStream", "Error in MJpegInputStream.read", ex)
//                return -1
//            }
//        }
//    }
//}
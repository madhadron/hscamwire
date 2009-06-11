{-# OPTIONS_GHC -XPatternSignatures #-}
-- | IIDC 1394 is a standard for industrial and scientific cameras
-- which attach to IEEE 1394 (a.k.a., Firewire) ports.  This is a
-- Haskell interface to the Camwire library for Linux to control
-- these cameras.  Camwire provides access only to Format 7 (image
-- size controlled by the user, not fixed by the size of the CCD),
-- which is essentially the only format anyone uses with modern
-- cameras, but to the full range of pixel encodings, from simple 8
-- bit monochrome to the 16 bit monochrome typical in microscopy and
-- the assorted exotic color encodings.
-- 
-- All possible cameras are attached to a "bus", initialized with the
-- 'initializeBus' function.  It returns uninitialized handles to all
-- IIDC 1394 cameras attached to the bus.  To initialize a specific
-- cameras, pass it to 'initializeCamera'.  You can get the
-- information the camera knows about itself with 'cameraId'.
-- 
-- Camwire supports a fairly general form of memory management: each
-- camera has some number of framebuffers (the number is determined by
-- the user) organized in a ring.  As it traverses the ring, it
-- overwrites whatever was found there before.  You can either copy
-- the data out of this framebuffer with a function such as
-- 'fetchFrameCopyAsMono16' or take a framebuffer temporarily out of
-- the ring while you work with it, then reinsert it when you're done
-- (with 'fetchFramebufferAsMono16' and 'freeFramebuffer').
-- 
-- Many of the functions return a value of type 'CamwireSuccess', which has two values: 'success' and 'failure'.  It is actually a binding around a C type of the same function.
module System.Camera.IIDC1394 where

import System.Camera.IIDC1394.Camwire

import Foreign.C.Error
import Foreign.C.String
import Foreign.C.Types
import Foreign.Ptr
import Foreign.ForeignPtr
import Foreign.Marshal.Alloc (malloc)
import Foreign.Marshal.Array (peekArray)
import Foreign
import System.Posix.Time
import System.Posix.Types
import Control.Monad (liftM)
import Data.Array.Storable

-- | The first action is always to initialize the IEEE 1394 bus to find all attached cameras.  The return value is a list of all the cameras detected, which may, of course, be empty.
initializeBus :: IO [CamwireUnmanagedHandle]
initializeBus = do
  len <- malloc
  rawHandles <- c_camwire_bus_create len
  numHandles <- liftM fromIntegral $ peek len
  peekArray numHandles rawHandles

destroyBus = c_camwire_bus_destroy

resetBus = c_camwire_bus_reset

-- | A handle returned by 'initializeBus' must be prepared for use.  If the initialization fails, 'Nothing' is returned.
initializeCamera :: CamwireUnmanagedHandle -> IO (Maybe CamwireHandle)
initializeCamera cam = do
  res <- c_camwire_create cam
  fnl <- wrap c_camwire_destroy
  if res == success then (newForeignPtr fnl cam >>= return . Just) else return Nothing

-- | 'CamwireId' returns the fields @vendor@, @model@, and @chip@ which the camera knows about itself.
cameraId :: CamwireHandle -> CamwireId
cameraId cam = unsafePerformIO $ do
  identifier <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_identifier c identifier
  peek identifier

-- | Camwire can attempt to maintain a model of the camera's internal state in memory rather than queryin the camera every time.  Doing so is called "shadowing".  It is generally fairly reliable, and can be useful if you need extra speed.
shadowed :: CamwireHandle -> IO Bool
shadowed cam = do
  sh <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_stateshadow c sh
  isSh <- peek sh
  return $ isSh == internalShadow

-- | Try to maintain a model of the camera in memory to avoid sending too many commands to it.
shadow :: CamwireHandle -> IO CamwireResult
shadow cam = withForeignPtr cam $ \c -> c_camwire_set_stateshadow c internalShadow

-- | Query the camera for every operation.
unshadow :: CamwireHandle -> IO CamwireResult
unshadow cam = withForeignPtr cam $ \c -> c_camwire_set_stateshadow c readFromHardware

-- | Returns the current number of framebuffers in memory used by the camera.
nFramebuffers :: CamwireHandle -> IO Int
nFramebuffers cam = do
  num <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_num_framebuffers c num
  peek num >>= return . fromIntegral

-- | Sets the current number of framebuffers.  Obviously, this needs to be strictly greater than 0.
setNFramebuffers :: CamwireHandle -> Int -> IO CamwireResult
setNFramebuffers cam n = withForeignPtr cam $ \c -> c_camwire_set_num_framebuffers c (fromIntegral n)

-- | Return how many frames behind the latest camera acquisition the next frame returned from the ringbuffer will be.  This number may change between the time you call this and the time you fetch the frame, so it's generally better to use the lag value returned from the actual fetch functions.
lag :: CamwireHandle -> IO Int
lag cam = do
  num <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_framebuffer_lag c num
  peek num >>= return . fromIntegral

-- | Tries to flush frames from the framebuffer, bringing the point to fetch next closer to the camera acquisition.  The number of frames to flush can be larger than the number of available frames or frame buffers, which will simply wipe all available frames.
flush :: CamwireHandle -> Int  -- ^ the number of frames to try to flush
      -> IO (Int,Int) -- ^ (the number flushed, the number of frames now lagging behind camera)
flush cam n = do
  let n' = fromIntegral n
  numFlushedPtr <- malloc
  numLaggingPtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_flush_framebuffers c n' 
                                         numFlushedPtr numLaggingPtr
  numFlushed <- liftM fromIntegral $ peek numFlushedPtr
  numLagging <- liftM fromIntegral $ peek numLaggingPtr
  return (numFlushed,numLagging)

-- | In Format 7, the area of the camera used is determined by the offset (the distance of one corner of the image from the corner of the camera) and frame size (the extent of the image beyond the offset).  'offset' returns (x,y), the positions of the offset from the corner.  'frameSize' returns the same structure for the extent beyond.  Both can be set with 'setOffset' and 'setFrameSize'.
offset :: CamwireHandle -> IO (Int,Int)
offset cam = do
  xPtr <- malloc
  yPtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_frame_offset c xPtr yPtr
  x <- liftM fromIntegral $ peek xPtr
  y <- liftM fromIntegral $ peek yPtr
  return (x,y)

setOffset :: CamwireHandle -> Int -- ^ x offset
          -> Int -- ^ y offset
          -> IO CamwireResult
setOffset cam x y = withForeignPtr cam $ \c -> 
                    c_camwire_set_frame_offset c (fromIntegral x) (fromIntegral y)

frameSize :: CamwireHandle -> IO (Int,Int)
frameSize cam = do
  wPtr <- malloc
  hPtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_frame_size c wPtr hPtr
  w <- liftM fromIntegral $ peek wPtr
  h <- liftM fromIntegral $ peek hPtr
  return (w,h)

setFrameSize :: CamwireHandle -> Int -> Int -> IO CamwireResult
setFrameSize cam w h = withForeignPtr cam $ \c ->
                       c_camwire_set_frame_size c (fromIntegral w) (fromIntegral h)

-- | The IIDC 1394 specification defines a large number of different encodings of pixels, from simply 8 and 16 bit monochrome packed into one and two bytes, to different arrangements of color sensors on the CCD.  'CamwirePixel' enumerates these possibilities:
-- 
-- * invalidPixel
-- * mono8
-- * yuv411
-- * yuv422
-- * yuv444
-- * rgb8
-- * mono16
-- * rgb16
-- * mono16s
-- * rgb16s
-- * raw8
-- * raw16
pixelCoding :: CamwireHandle -> IO CamwirePixel
pixelCoding cam = do
  pixelPtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_pixel_coding c pixelPtr
  peek pixelPtr

setPixelCoding :: CamwireHandle -> CamwirePixel -> IO CamwireResult
setPixelCoding cam px = withForeignPtr cam $ \c ->
                        c_camwire_set_pixel_coding c px

-- | In color cameras, the layout of the different color sensors varies.  The specification defines several possible tilings:
-- 
-- * invalidTiling
-- * rggbTiling
-- * gbrgTiling
-- * grbgTiling
-- * bggrTiling
-- * uyvyTiling
-- * yuyvTiling
pixelTiling :: CamwireHandle -> IO CamwireTiling
pixelTiling cam = do
  tilingPtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_pixel_tiling c tilingPtr
  peek tilingPtr

-- | This is a utility function that gives the bit depth of any of the 'CamwirePixel' types.
pixelDepth :: CamwirePixel -> Int
pixelDepth px = unsafePerformIO $ do
  depthPtr <- malloc
  return $ c_camwire_pixel_depth px depthPtr
  liftM fromIntegral $ peek depthPtr

-- | Frame rate is the number of frames captured per second when a camera is run in continuous capture (as opposed to one-shot) mode.
framerate :: CamwireHandle -> IO Double
framerate cam = do
  frameratePtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_framerate c frameratePtr
  liftM realToFrac $ peek frameratePtr

setFramerate :: CamwireHandle -> Double -> IO CamwireResult
setFramerate cam framerate = withForeignPtr cam $ \c ->
                             c_camwire_set_framerate c (realToFrac framerate)

-- | The exposure time is the length of time light is allowed to fall on the CCD in each frame, measured in seconds.
exposureTime :: CamwireHandle -> IO Double
exposureTime cam = do
  timePtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_shutter c timePtr
  liftM realToFrac $ peek timePtr

setExposureTime :: CamwireHandle -> Double -> IO CamwireResult
setExposureTime cam time = withForeignPtr cam $ \c ->
                           c_camwire_set_shutter c (realToFrac time)

-- | Cameras may be triggered internally by their own timers ('internalTrigger'), or hooked to an external signal as their trigger ('externalTrigger'), for instance to synchronize them with illumination sources.  For external triggers, you must also set the trigger polarity, that is, whether the camera triggers on a sudden increase in voltage ('risingEdgeTrigger' or its synonym 'activeHighTrigger'), or a sudden decrease ('fallingEdgeTrigger' and its synonym 'activeLowTrigger').
triggerSource :: CamwireHandle -> IO CameraTriggerSource
triggerSource cam = do
  trigPtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_trigger_source c trigPtr
  peek trigPtr

setTriggerSource :: CamwireHandle -> CameraTriggerSource -> IO CamwireResult
setTriggerSource cam src = withForeignPtr cam $ \c -> c_camwire_set_trigger_source c src

triggerPolarity :: CamwireHandle -> IO CameraTriggerPolarity
triggerPolarity cam = do
  polPtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_trigger_polarity c polPtr
  peek polPtr

setTriggerPolarity :: CamwireHandle -> CameraTriggerPolarity -> IO CamwireResult
setTriggerPolarity cam pol = withForeignPtr cam $ \c -> c_camwire_set_trigger_polarity c pol

-- | IIDC 1394 assumes cameras are linear response, that is, twice as many photons (within the dynamic range of the camera) gives twice as many grey levels.  The gain of the camera sets that slope.  A given camera has a minimum and maximum gain fixed by its hardware.  The arguments passed to 'gain' and 'setGain' are floating point numbers between 0 and 1 which sets a slope between the camera's minimum and maximum.  The slope is chosen linearly, so for a camera with minimum slope N and maximum slope M, the resulting slope for a given gain is (M-N)*gain + N.  The actual value may not be precisely what you request, but the nearest level the camera supports.
gain :: CamwireHandle -> IO Double
gain cam = do
  gainPtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_gain c gainPtr
  liftM realToFrac $ peek gainPtr

setGain :: CamwireHandle -> Double -> IO CamwireResult
setGain cam g = withForeignPtr cam $ \c -> c_camwire_set_gain c (realToFrac g)

-- | Sets the brightness of the camera within a range of -1.0 (darkest) to +1.0 (brightest).  This is then interpolated linearly in the camera's real brightness range, so for a camera with minimum brightness N and maximum brightness M, the real brightness level is the closest brightness to (M-N)*(brightness + 1.0)/2.0 supported by the camera.
brightness :: CamwireHandle -> IO Double
brightness cam = do
  bPtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_brightness c bPtr
  liftM realToFrac $ peek bPtr

setBrightness :: CamwireHandle -> Double -> IO CamwireResult
setBrightness cam b = withForeignPtr cam $ \c -> c_camwire_set_brightness c (realToFrac b)

data WhiteBalance = WhiteBalance { red, blue :: Double }

-- | The white balance sets the gain of the red and blue channels of a color camera, on a scale from 0 (minimum gain for that color) to 1 (maximum gain for that color).
whiteBalance :: CamwireHandle -> IO WhiteBalance
whiteBalance cam = do
  redPtr <- malloc
  bluePtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_white_balance c redPtr bluePtr
  red <- liftM realToFrac $ peek redPtr
  blue <- liftM realToFrac $ peek bluePtr
  return $ WhiteBalance red blue

setWhiteBalance :: CamwireHandle -> WhiteBalance -> IO CamwireResult
setWhiteBalance cam wb = withForeignPtr cam $ \c -> c_camwire_set_white_balance c 
                         (realToFrac $ red wb) (realToFrac $ blue wb)

-- | A camera can be run either in 'singleShot' (when told to 'run', capture one frame, and stop again) or 'continouousCapture' (when told to 'run', continue capturing to the ring of framebuffers until told explicitly to stop).
acquisitionMode :: CamwireHandle -> IO AcquisitionMode
acquisitionMode cam = do
  modePtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_single_shot c modePtr
  peek modePtr

setAcquisitionMode :: CamwireHandle -> AcquisitionMode -> IO CamwireResult
setAcquisitionMode cam mode = withForeignPtr cam $ \c -> c_camwire_set_single_shot c mode

-- | Cameras may be running ('cameraRunning') or stopped ('cameraStopped').
running :: CamwireHandle -> IO Bool
running cam = do
  rPtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_run_stop c rPtr
  r <- peek rPtr
  return (r == cameraRunning)

-- | Start a camera running.  In single shot mode, captures a single frame, and stops again.  In continuous capture mode, keeps capturing until told explicitly to 'stop'.
run :: CamwireHandle -> IO CamwireResult
run cam = withForeignPtr cam $ \c -> c_camwire_set_run_stop c cameraRunning

-- | Stop capture.  Probably does nothing in single shot mode.
stop :: CamwireHandle -> IO CamwireResult
stop cam = withForeignPtr cam $ \c -> c_camwire_set_run_stop c cameraStopped

transpose arr = do
  ((x0,y0), (xf,yf)) <- getBounds arr
  mapIndices ((y0,x0), (yf,xf)) (\(x,y) -> (y,x)) arr

-- | For any given pixel encoding (and at the moment this binding only supports mono16), there are four functions to get a captured frame.  Captured frames are always returned as @StorableArray (Int,Int)@, along with an integer giving the number of frames behind the latest capture by the camera this frame is.
-- 
-- * 'fetchFrameCopyAsMono16' returns a new block of memory containing the image, independent of the camera's framebuffers.
-- * 'copyFrameAsMono16' does the same as 'fetchFrameCopyAsMono16', but puts the image into a given array instead.  It is up to the user to make sure the provided array has the correct size.
-- * 'fetchFramebufferAsMono16' lifts the framebuffer in question out of the camera's ringbuffer, and returns a pointer to it.  When you are finished with it, it should be garbage collected automatically and returned to the ring, or can be explicitly returned with 'freeFramebuffer'.  If there is no frame ready to take, it blocks and waits until the next capture.
-- * 'immediateFetchFramebufferAsMono16' is the same as 'fetchFramebufferAsMono16', but doesn't block.  If there is no waiting frame, it returns 'Nothing' for the array.
fetchFrameCopyAsMono16 :: CamwireHandle -> IO (StorableArray (Int,Int) Word16, Int)
fetchFrameCopyAsMono16 cam = do
  (w,h) <- frameSize cam
  lagPtr <- malloc
  framePtr :: ForeignPtr Word16 <- mallocForeignPtrArray (w*h)
  withForeignPtr framePtr $ \p ->
      withForeignPtr cam $ \c -> c_camwire_copy_next_frame c (castPtr p) lagPtr
  lag <- liftM fromIntegral $ peek lagPtr
  frameArray <- unsafeForeignPtrToStorableArray framePtr ((0,0), (w-1,h-1)) >>= transpose
  return (frameArray, lag)

copyFrameAsMono16 :: CamwireHandle -> StorableArray (Int,Int) Word16 -> IO Int
copyFrameAsMono16 cam arr = do
  lagPtr <- malloc
  withForeignPtr cam $ \c ->
      withStorableArray arr $ \a ->
          c_camwire_copy_next_frame c (castPtr a) lagPtr
  liftM fromIntegral $ peek lagPtr

fetchFramebufferAsMono16 :: CamwireHandle -> IO (StorableArray (Int,Int) Word16, Int)
fetchFramebufferAsMono16 cam = do
  (w,h) <- frameSize cam
  lagPtr <- malloc
  framePtrPtr :: Ptr (Ptr ()) <- malloc
  res <- withForeignPtr cam $ \c -> c_camwire_point_next_frame c framePtrPtr lagPtr
  if res == failure then error "Camwire.hs, line 252: Failed to fetch a reference to a DMA framebuffer." else return ()
  lag <- liftM fromIntegral $ peek lagPtr
  framePtr <- liftM castPtr $ peek framePtrPtr
  unpt <- wrapWord16 (\_ -> withForeignPtr cam $ \c -> c_camwire_unpoint_frame c >> return ())
  frameFPtr <- newForeignPtr unpt framePtr
  frameArray <- unsafeForeignPtrToStorableArray frameFPtr ((0,0), (w-1,h-1)) >>= transpose
  return (frameArray, lag)

immediateFetchFramebufferAsMono16 :: CamwireHandle -> IO (Maybe (StorableArray (Int,Int) Word16), Int)
immediateFetchFramebufferAsMono16 cam = do
  (w,h) <- frameSize cam
  lagPtr <- malloc
  framePtrPtr :: Ptr (Ptr ()) <- malloc
  res <- withForeignPtr cam $ \c -> c_camwire_point_next_frame c framePtrPtr lagPtr
  if res == failure then error "Camwire.hs, line 252: Failed to fetch a reference to a DMA framebuffer." else return ()
  lag <- liftM fromIntegral $ peek lagPtr
  if lag == 0 then return (Nothing, 0) else do
      framePtr <- liftM castPtr $ peek framePtrPtr
      unpt <- wrapWord16 (\_ -> withForeignPtr cam $ \c -> c_camwire_unpoint_frame c >> return ())
      frameFPtr <- newForeignPtr unpt framePtr
      frameArray <- unsafeForeignPtrToStorableArray frameFPtr ((0,0), (w-1,h-1)) >>= transpose
      return (Just frameArray, lag)

freeFramebuffer :: CamwireHandle -> IO CamwireResult
freeFramebuffer cam = withForeignPtr cam $ \c -> c_camwire_unpoint_frame c

-- | Returns the number of the last frame 
frameNumber :: CamwireHandle -> IO Integer
frameNumber cam = do
  fPtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_framenumber c fPtr
  liftM fromIntegral $ peek fPtr

-- | Returns the timestamp of the frame returned by any of the framebuffer access commands (i.e., 'copyFrameAsMono16').  It is returned as a 'Timespec' which has two fields: 'seconds' and 'nanoseconds'.
timestamp :: CamwireHandle -> IO Timespec
timestamp cam = do
  tPtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_timestamp c tPtr
  peek tPtr
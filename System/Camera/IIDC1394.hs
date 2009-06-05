{-# OPTIONS_GHC -XPatternSignatures #-}

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

initializeBus :: IO [CamwireUnmanagedHandle]
initializeBus = do
  len <- malloc
  rawHandles <- c_camwire_bus_create len
  numHandles <- liftM fromIntegral $ peek len
  peekArray numHandles rawHandles

destroyBus = c_camwire_bus_destroy

resetBus = c_camwire_bus_reset

initializeCamera :: CamwireUnmanagedHandle -> IO (Maybe CamwireHandle)
initializeCamera cam = do
  res <- c_camwire_create cam
  fnl <- wrap c_camwire_destroy
  if res == success then (newForeignPtr fnl cam >>= return . Just) else return Nothing

cameraId :: CamwireHandle -> CamwireId
cameraId cam = unsafePerformIO $ do
  identifier <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_identifier c identifier
  peek identifier

shadowed :: CamwireHandle -> IO Bool
shadowed cam = do
  sh <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_stateshadow c sh
  isSh <- peek sh
  return $ isSh == internalShadow

shadow :: CamwireHandle -> IO CamwireResult
shadow cam = withForeignPtr cam $ \c -> c_camwire_set_stateshadow c internalShadow

unshadow :: CamwireHandle -> IO CamwireResult
unshadow cam = withForeignPtr cam $ \c -> c_camwire_set_stateshadow c readFromHardware

nFramebuffers :: CamwireHandle -> IO Int
nFramebuffers cam = do
  num <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_num_framebuffers c num
  peek num >>= return . fromIntegral

setNFramebuffers :: CamwireHandle -> Int -> IO CamwireResult
setNFramebuffers cam n = withForeignPtr cam $ \c -> c_camwire_set_num_framebuffers c (fromIntegral n)

lag :: CamwireHandle -> IO Int
lag cam = do
  num <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_framebuffer_lag c num
  peek num >>= return . fromIntegral

flush :: CamwireHandle -> Int -> IO (Int,Int)
flush cam n = do
  let n' = fromIntegral n
  numFlushedPtr <- malloc
  numLaggingPtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_flush_framebuffers c n' 
                                         numFlushedPtr numLaggingPtr
  numFlushed <- liftM fromIntegral $ peek numFlushedPtr
  numLagging <- liftM fromIntegral $ peek numLaggingPtr
  return (numFlushed,numLagging)

offset :: CamwireHandle -> IO (Int,Int)
offset cam = do
  xPtr <- malloc
  yPtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_frame_offset c xPtr yPtr
  x <- liftM fromIntegral $ peek xPtr
  y <- liftM fromIntegral $ peek yPtr
  return (x,y)

setOffset :: CamwireHandle -> Int -> Int -> IO CamwireResult
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

pixelCoding :: CamwireHandle -> IO CamwirePixel
pixelCoding cam = do
  pixelPtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_pixel_coding c pixelPtr
  peek pixelPtr

setPixelCoding :: CamwireHandle -> CamwirePixel -> IO CamwireResult
setPixelCoding cam px = withForeignPtr cam $ \c ->
                        c_camwire_set_pixel_coding c px

pixelTiling :: CamwireHandle -> IO CamwireTiling
pixelTiling cam = do
  tilingPtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_pixel_tiling c tilingPtr
  peek tilingPtr

pixelDepth :: CamwirePixel -> Int
pixelDepth px = unsafePerformIO $ do
  depthPtr <- malloc
  return $ c_camwire_pixel_depth px depthPtr
  liftM fromIntegral $ peek depthPtr

framerate :: CamwireHandle -> IO Double
framerate cam = do
  frameratePtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_framerate c frameratePtr
  liftM realToFrac $ peek frameratePtr

setFramerate :: CamwireHandle -> Double -> IO CamwireResult
setFramerate cam framerate = withForeignPtr cam $ \c ->
                             c_camwire_set_framerate c (realToFrac framerate)

exposureTime :: CamwireHandle -> IO Double
exposureTime cam = do
  timePtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_shutter c timePtr
  liftM realToFrac $ peek timePtr

setExposureTime :: CamwireHandle -> Double -> IO CamwireResult
setExposureTime cam time = withForeignPtr cam $ \c ->
                           c_camwire_set_shutter c (realToFrac time)

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

gain :: CamwireHandle -> IO Double
gain cam = do
  gainPtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_gain c gainPtr
  liftM realToFrac $ peek gainPtr

setGain :: CamwireHandle -> Double -> IO CamwireResult
setGain cam g = withForeignPtr cam $ \c -> c_camwire_set_gain c (realToFrac g)

brightness :: CamwireHandle -> IO Double
brightness cam = do
  bPtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_brightness c bPtr
  liftM realToFrac $ peek bPtr

setBrightness :: CamwireHandle -> Double -> IO CamwireResult
setBrightness cam b = withForeignPtr cam $ \c -> c_camwire_set_brightness c (realToFrac b)

data WhiteBalance = WhiteBalance { red, blue :: Double }

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

acquisitionMode :: CamwireHandle -> IO AcquisitionMode
acquisitionMode cam = do
  modePtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_single_shot c modePtr
  peek modePtr

setAcquisitionMode :: CamwireHandle -> AcquisitionMode -> IO CamwireResult
setAcquisitionMode cam mode = withForeignPtr cam $ \c -> c_camwire_set_single_shot c mode

running :: CamwireHandle -> IO Bool
running cam = do
  rPtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_run_stop c rPtr
  r <- peek rPtr
  return (r == cameraRunning)

run :: CamwireHandle -> IO CamwireResult
run cam = withForeignPtr cam $ \c -> c_camwire_set_run_stop c cameraRunning

stop :: CamwireHandle -> IO CamwireResult
stop cam = withForeignPtr cam $ \c -> c_camwire_set_run_stop c cameraStopped

transpose arr = do
  ((x0,y0), (xf,yf)) <- getBounds arr
  mapIndices ((y0,x0), (yf,xf)) (\(x,y) -> (y,x)) arr

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

frameNumber :: CamwireHandle -> IO Integer
frameNumber cam = do
  fPtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_framenumber c fPtr
  liftM fromIntegral $ peek fPtr

timestamp :: CamwireHandle -> IO Timespec
timestamp cam = do
  tPtr <- malloc
  withForeignPtr cam $ \c -> c_camwire_get_timestamp c tPtr
  peek tPtr
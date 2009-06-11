{-# INCLUDE <camwire/camwire.h> #-}
{-# INCLUDE <camwire/camwirebus.h> #-}
{-# INCLUDE <time.h> #-}
{-# LINE 1 "System/Camera/IIDC1394/Camwire.hsc" #-}
{-# LANGUAGE ForeignFunctionInterface, EmptyDataDecls, ScopedTypeVariables #-}
{-# LINE 2 "System/Camera/IIDC1394/Camwire.hsc" #-}
{-# OPTIONS_GHC -fno-warn-unused-imports -fno-warn-unused-binds -XGeneralizedNewtypeDeriving #-}

module System.Camera.IIDC1394.Camwire where

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
import Data.Time.Clock
import Data.Time.Clock.POSIX


{-# LINE 21 "System/Camera/IIDC1394/Camwire.hsc" #-}

{-# LINE 22 "System/Camera/IIDC1394/Camwire.hsc" #-}

{-# LINE 23 "System/Camera/IIDC1394/Camwire.hsc" #-}

data CamwireHandleT
type CamwireUnmanagedHandle = Ptr CamwireHandleT
type CamwireHandle = ForeignPtr CamwireHandleT

foreign import ccall "camwirebus.h camwire_bus_create" c_camwire_bus_create :: Ptr CInt -> IO (Ptr CamwireUnmanagedHandle)

foreign import ccall "camwirebus.h camwire_bus_destroy" c_camwire_bus_destroy :: IO ()

foreign import ccall "camwirebus.h camwire_bus_reset" c_camwire_bus_reset :: IO ()


data CamwireId = CamwireId { vendor :: String,
                             model :: String,
                             chip :: String }
               deriving (Eq,Show,Read)


{-# LINE 41 "System/Camera/IIDC1394/Camwire.hsc" #-}

instance Storable CamwireId where
    alignment _ = 1
{-# LINE 44 "System/Camera/IIDC1394/Camwire.hsc" #-}
    sizeOf _ = (303)
{-# LINE 45 "System/Camera/IIDC1394/Camwire.hsc" #-}
    peek ptr = do
      v <- peekCString $ (\hsc_ptr -> hsc_ptr `plusPtr` 0) ptr
{-# LINE 47 "System/Camera/IIDC1394/Camwire.hsc" #-}
      m <- peekCString $ (\hsc_ptr -> hsc_ptr `plusPtr` 101) ptr
{-# LINE 48 "System/Camera/IIDC1394/Camwire.hsc" #-}
      c <- peekCString $ (\hsc_ptr -> hsc_ptr `plusPtr` 202) ptr
{-# LINE 49 "System/Camera/IIDC1394/Camwire.hsc" #-}
      return $ CamwireId { vendor = v, model = m, chip = c }
    poke ptr (CamwireId v m c) = do
      withCStringLen (take 100 v) $
{-# LINE 52 "System/Camera/IIDC1394/Camwire.hsc" #-}
                     uncurry (copyArray $ (\hsc_ptr -> hsc_ptr `plusPtr` 0) ptr)
{-# LINE 53 "System/Camera/IIDC1394/Camwire.hsc" #-}
      withCStringLen (take 100 m) $
{-# LINE 54 "System/Camera/IIDC1394/Camwire.hsc" #-}
                     uncurry (copyArray $ (\hsc_ptr -> hsc_ptr `plusPtr` 101) ptr)
{-# LINE 55 "System/Camera/IIDC1394/Camwire.hsc" #-}
      withCStringLen (take 100 c) $
{-# LINE 56 "System/Camera/IIDC1394/Camwire.hsc" #-}
                     uncurry (copyArray $ (\hsc_ptr -> hsc_ptr `plusPtr` 202) ptr)
{-# LINE 57 "System/Camera/IIDC1394/Camwire.hsc" #-}

newtype CamwirePixel = CamwirePixel { pixelType :: CInt }
    deriving (Eq,Show,Storable)

invalidPixel  :: CamwirePixel
invalidPixel  = CamwirePixel 0
mono8  :: CamwirePixel
mono8  = CamwirePixel 1
yuv411  :: CamwirePixel
yuv411  = CamwirePixel 2
yuv422  :: CamwirePixel
yuv422  = CamwirePixel 3
yuv444  :: CamwirePixel
yuv444  = CamwirePixel 4
rgb8  :: CamwirePixel
rgb8  = CamwirePixel 5
mono16  :: CamwirePixel
mono16  = CamwirePixel 6
rgb16  :: CamwirePixel
rgb16  = CamwirePixel 7
mono16s  :: CamwirePixel
mono16s  = CamwirePixel 8
rgb16s  :: CamwirePixel
rgb16s  = CamwirePixel 9
raw8  :: CamwirePixel
raw8  = CamwirePixel 10
raw16  :: CamwirePixel
raw16  = CamwirePixel 11

{-# LINE 74 "System/Camera/IIDC1394/Camwire.hsc" #-}

newtype CamwireTiling = CamwireTiling { tilingType :: CInt }
    deriving (Eq,Show,Storable)
invalidTiling  :: CamwireTiling
invalidTiling  = CamwireTiling 0
rggbTiling  :: CamwireTiling
rggbTiling  = CamwireTiling 1
gbrgTiling  :: CamwireTiling
gbrgTiling  = CamwireTiling 2
grbgTiling  :: CamwireTiling
grbgTiling  = CamwireTiling 3
bggrTiling  :: CamwireTiling
bggrTiling  = CamwireTiling 4
uyvyTiling  :: CamwireTiling
uyvyTiling  = CamwireTiling 5
yuyvTiling  :: CamwireTiling
yuyvTiling  = CamwireTiling 6

{-# LINE 85 "System/Camera/IIDC1394/Camwire.hsc" #-}

newtype CamwireResult = CamwireResult { resultOf :: CInt }
    deriving (Eq,Show,Storable)

success :: CamwireResult
success = CamwireResult 0
{-# LINE 91 "System/Camera/IIDC1394/Camwire.hsc" #-}
failure :: CamwireResult
failure = CamwireResult 1
{-# LINE 93 "System/Camera/IIDC1394/Camwire.hsc" #-}

data CamwireState
data CamwireConf

foreign import ccall unsafe "camwire.h camwire_create" c_camwire_create :: CamwireUnmanagedHandle -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_create_from_struct" c_camwire_create_from_struct :: CamwireUnmanagedHandle -> Ptr CamwireState -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_destroy" c_camwire_destroy :: CamwireUnmanagedHandle -> IO ()

foreign import ccall "wrapper" wrap :: (CamwireUnmanagedHandle -> IO ()) -> IO (FunPtr (CamwireUnmanagedHandle -> IO ()))

foreign import ccall unsafe "camwire.h camwire_get_state" c_camwire_get_state :: CamwireUnmanagedHandle -> Ptr CamwireState -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_set_state" c_camwire_set_state :: CamwireUnmanagedHandle -> Ptr CamwireState -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_read_state_from_file" c_camwire_read_state_from_file :: CString -> Ptr CamwireState -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_write_state_to_file" c_camwire_write_state_to_file :: CString -> Ptr CamwireState -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_get_config" c_camwire_get_config :: CamwireUnmanagedHandle -> Ptr CamwireConf -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_print_config" c_camwire_print_config :: Ptr CamwireConf -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_get_identifier" c_camwire_get_identifier :: CamwireUnmanagedHandle -> Ptr CamwireId -> IO CamwireResult

newtype StateShadowStatus = StateShadowStatus { getStatus :: CInt } 
    deriving (Eq,Show,Storable)
internalShadow :: StateShadowStatus
internalShadow = StateShadowStatus 1
readFromHardware :: StateShadowStatus
readFromHardware = StateShadowStatus 0

foreign import ccall unsafe "camwire.h camwire_get_stateshadow" c_camwire_get_stateshadow :: CamwireUnmanagedHandle -> Ptr StateShadowStatus -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_set_stateshadow" c_camwire_set_stateshadow :: CamwireUnmanagedHandle -> StateShadowStatus -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_get_num_framebuffers" c_camwire_get_num_framebuffers :: CamwireUnmanagedHandle -> Ptr CInt -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_set_num_framebuffers" c_camwire_set_num_framebuffers :: CamwireUnmanagedHandle -> CInt -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_get_framebuffer_lag" c_camwire_get_framebuffer_lag :: CamwireUnmanagedHandle -> Ptr CInt -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_flush_framebuffers" c_camwire_flush_framebuffers :: CamwireUnmanagedHandle -> CInt -> Ptr CInt -> Ptr CInt -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_get_frame_offset" c_camwire_get_frame_offset :: CamwireUnmanagedHandle -> Ptr CInt -> Ptr CInt -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_set_frame_offset" c_camwire_set_frame_offset :: CamwireUnmanagedHandle -> CInt -> CInt -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_get_frame_size" c_camwire_get_frame_size :: CamwireUnmanagedHandle -> Ptr CInt -> Ptr CInt -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_set_frame_size" c_camwire_set_frame_size :: CamwireUnmanagedHandle -> CInt -> CInt -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_get_pixel_coding" c_camwire_get_pixel_coding :: CamwireUnmanagedHandle -> Ptr CamwirePixel -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_set_pixel_coding" c_camwire_set_pixel_coding :: CamwireUnmanagedHandle -> CamwirePixel -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_get_pixel_tiling" c_camwire_get_pixel_tiling :: CamwireUnmanagedHandle -> Ptr CamwireTiling -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_pixel_depth" c_camwire_pixel_depth :: CamwirePixel -> Ptr CInt -> CamwireResult

foreign import ccall unsafe "camwire.h camwire_get_framerate" c_camwire_get_framerate :: CamwireUnmanagedHandle -> Ptr CDouble -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_set_framerate" c_camwire_set_framerate :: CamwireUnmanagedHandle -> CDouble -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_get_shutter" c_camwire_get_shutter :: CamwireUnmanagedHandle -> Ptr CDouble -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_set_shutter" c_camwire_set_shutter :: CamwireUnmanagedHandle -> CDouble -> IO CamwireResult

newtype CameraTriggerSource = CameraTriggerSource { triggerSource :: CInt }
    deriving (Eq,Show,Storable)
internalTrigger :: CameraTriggerSource
internalTrigger = CameraTriggerSource 0
externalTrigger :: CameraTriggerSource
externalTrigger = CameraTriggerSource 1

foreign import ccall unsafe "camwire.h camwire_get_trigger_source" c_camwire_get_trigger_source :: CamwireUnmanagedHandle -> Ptr CameraTriggerSource -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_set_trigger_source" c_camwire_set_trigger_source :: CamwireUnmanagedHandle -> CameraTriggerSource -> IO CamwireResult

newtype CameraTriggerPolarity = CameraTriggerPolarity { triggerPolarity :: CInt }
    deriving (Eq,Show,Storable)
risingEdgeTrigger :: CameraTriggerPolarity
risingEdgeTrigger = CameraTriggerPolarity 1
fallingEdgeTrigger :: CameraTriggerPolarity
fallingEdgeTrigger = CameraTriggerPolarity 0
activeHighTrigger :: CameraTriggerPolarity
activeHighTrigger = CameraTriggerPolarity 1
activeLowTrigger :: CameraTriggerPolarity
activeLowTrigger = CameraTriggerPolarity 0

foreign import ccall unsafe "camwire.h camwire_get_trigger_polarity" c_camwire_get_trigger_polarity :: CamwireUnmanagedHandle -> Ptr CameraTriggerPolarity -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_set_trigger_polarity" c_camwire_set_trigger_polarity :: CamwireUnmanagedHandle -> CameraTriggerPolarity -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_get_gain" c_camwire_get_gain :: CamwireUnmanagedHandle -> Ptr CDouble -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_set_gain" c_camwire_set_gain :: CamwireUnmanagedHandle -> CDouble -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_get_brightness" c_camwire_get_brightness :: CamwireUnmanagedHandle -> Ptr CDouble -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_set_brightness" c_camwire_set_brightness :: CamwireUnmanagedHandle -> CDouble -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_get_white_balance" c_camwire_get_white_balance :: CamwireUnmanagedHandle -> Ptr CDouble -> Ptr CDouble -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_set_white_balance" c_camwire_set_white_balance :: CamwireUnmanagedHandle -> CDouble -> CDouble -> IO CamwireResult

newtype AcquisitionMode = AcquisitionMode { shotState :: CInt }
    deriving (Eq,Show,Storable)
singleShot :: AcquisitionMode
singleShot = AcquisitionMode 1
continuousCapture :: AcquisitionMode
continuousCapture = AcquisitionMode 0

foreign import ccall unsafe "camwire.h camwire_get_single_shot" c_camwire_get_single_shot :: CamwireUnmanagedHandle -> Ptr AcquisitionMode -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_set_single_shot" c_camwire_set_single_shot :: CamwireUnmanagedHandle -> AcquisitionMode -> IO CamwireResult

newtype CameraRunning = CameraRunning CInt deriving (Eq,Show,Storable)
cameraRunning :: CameraRunning
cameraRunning = CameraRunning 1
cameraStopped :: CameraRunning
cameraStopped = CameraRunning 0

foreign import ccall unsafe "camwire.h camwire_get_run_stop" c_camwire_get_run_stop :: CamwireUnmanagedHandle -> Ptr CameraRunning -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_set_run_stop" c_camwire_set_run_stop :: CamwireUnmanagedHandle -> CameraRunning -> IO CamwireResult

-- The raw functions for getting images don't know anything about the type
-- of data they are returning, so I use Ptr () for the frame buffer.
-- The wrapper around this will make them into appropriate StorableArray's.

foreign import ccall unsafe "camwire.h camwire_copy_next_frame" c_camwire_copy_next_frame :: CamwireUnmanagedHandle -> Ptr () -> Ptr CInt -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_point_next_frame" c_camwire_point_next_frame :: CamwireUnmanagedHandle -> Ptr (Ptr ()) -> Ptr CInt -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_point_next_frame_poll" c_camwire_point_next_frame_poll :: CamwireUnmanagedHandle -> Ptr (Ptr ()) -> Ptr CInt -> IO CamwireResult

foreign import ccall "wrapper" wrapWord16 :: (Ptr Word16 -> IO ()) -> IO (FunPtr (Ptr Word16 -> IO ()))

foreign import ccall unsafe "camwire.h camwire_unpoint_frame" c_camwire_unpoint_frame :: CamwireUnmanagedHandle -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_get_framenumber" c_camwire_get_framenumber :: CamwireUnmanagedHandle -> Ptr CLong -> IO CamwireResult

data Timespec = Timespec { seconds :: CTime, nanoseconds :: CLong }
               deriving (Eq,Show)

instance Storable Timespec where
    alignment _ = 4
{-# LINE 242 "System/Camera/IIDC1394/Camwire.hsc" #-}
    sizeOf _ = (8)
{-# LINE 243 "System/Camera/IIDC1394/Camwire.hsc" #-}
    peek ptr = do
      s :: CTime <- (\hsc_ptr -> peekByteOff hsc_ptr 0) ptr
{-# LINE 245 "System/Camera/IIDC1394/Camwire.hsc" #-}
      ns :: CLong <- (\hsc_ptr -> peekByteOff hsc_ptr 4) ptr
{-# LINE 246 "System/Camera/IIDC1394/Camwire.hsc" #-}
      return (Timespec { seconds = s, nanoseconds = ns })
    poke ptr ts = do
      (\hsc_ptr -> pokeByteOff hsc_ptr 0) ptr (seconds ts)
{-# LINE 249 "System/Camera/IIDC1394/Camwire.hsc" #-}
      (\hsc_ptr -> pokeByteOff hsc_ptr 4) ptr (nanoseconds ts)
{-# LINE 250 "System/Camera/IIDC1394/Camwire.hsc" #-}

foreign import ccall unsafe "camwire.h camwire_get_timestamp" c_camwire_get_timestamp :: CamwireUnmanagedHandle -> Ptr Timespec -> IO CamwireResult

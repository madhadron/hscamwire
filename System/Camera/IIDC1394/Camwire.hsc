{-# LANGUAGE ForeignFunctionInterface, EmptyDataDecls #-}
{-# OPTIONS_GHC -fno-warn-unused-imports -fno-warn-unused-binds -XGeneralizedNewtypeDeriving -XPatternSignatures #-}

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

#include <camwire/camwire.h>
#include <camwire/camwirebus.h>
#include <time.h>

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

#let alignment t = "%lu", (unsigned long)offsetof(struct {char x__; t (y__); }, y__)

instance Storable CamwireId where
    alignment _ = #{alignment Camwire_id}
    sizeOf _ = #{size Camwire_id}
    peek ptr = do
      v <- peekCString $ #{ptr Camwire_id, vendor} ptr
      m <- peekCString $ #{ptr Camwire_id, model} ptr
      c <- peekCString $ #{ptr Camwire_id, chip} ptr
      return $ CamwireId { vendor = v, model = m, chip = c }
    poke ptr (CamwireId v m c) = do
      withCStringLen (take #{const CAMWIRE_ID_MAX_CHARS} v) $
                     uncurry (copyArray $ #{ptr Camwire_id, vendor} ptr)
      withCStringLen (take #{const CAMWIRE_ID_MAX_CHARS} m) $
                     uncurry (copyArray $ #{ptr Camwire_id, model} ptr)
      withCStringLen (take #{const CAMWIRE_ID_MAX_CHARS} c) $
                     uncurry (copyArray $ #{ptr Camwire_id, chip} ptr)

newtype CamwirePixel = CamwirePixel { pixelType :: CInt }
    deriving (Eq,Show,Storable)

#{enum CamwirePixel, CamwirePixel, 
       invalidPixel = CAMWIRE_PIXEL_INVALID,
       mono8 = CAMWIRE_PIXEL_MONO8,
       yuv411 = CAMWIRE_PIXEL_YUV411,
       yuv422 = CAMWIRE_PIXEL_YUV422,
       yuv444 = CAMWIRE_PIXEL_YUV444,
       rgb8 = CAMWIRE_PIXEL_RGB8,
       mono16 = CAMWIRE_PIXEL_MONO16,
       rgb16 = CAMWIRE_PIXEL_RGB16,
       mono16s = CAMWIRE_PIXEL_MONO16S,
       rgb16s = CAMWIRE_PIXEL_RGB16S,
       raw8 = CAMWIRE_PIXEL_RAW8,
       raw16 = CAMWIRE_PIXEL_RAW16 }

newtype CamwireTiling = CamwireTiling { tilingType :: CInt }
    deriving (Eq,Show,Storable)
#{enum CamwireTiling, CamwireTiling,
       invalidTiling = CAMWIRE_TILING_INVALID,
       rggbTiling = CAMWIRE_TILING_RGGB,
       gbrgTiling = CAMWIRE_TILING_GBRG,
       grbgTiling = CAMWIRE_TILING_GRBG,
       bggrTiling = CAMWIRE_TILING_BGGR,
       uyvyTiling = CAMWIRE_TILING_UYVY,
       yuyvTiling = CAMWIRE_TILING_YUYV }

newtype CamwireResult = CamwireResult { resultOf :: CInt }
    deriving (Eq,Show,Storable)

success = CamwireResult #const CAMWIRE_SUCCESS
failure = CamwireResult #const CAMWIRE_FAILURE

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
internalShadow = StateShadowStatus 1
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
internalTrigger = CameraTriggerSource 0
externalTrigger = CameraTriggerSource 1

foreign import ccall unsafe "camwire.h camwire_get_trigger_source" c_camwire_get_trigger_source :: CamwireUnmanagedHandle -> Ptr CameraTriggerSource -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_set_trigger_source" c_camwire_set_trigger_source :: CamwireUnmanagedHandle -> CameraTriggerSource -> IO CamwireResult

newtype CameraTriggerPolarity = CameraTriggerPolarity { triggerPolarity :: CInt }
    deriving (Eq,Show,Storable)
risingEdgeTrigger = CameraTriggerPolarity 1
fallingEdgeTrigger = CameraTriggerPolarity 0
activeHighTrigger = CameraTriggerPolarity 1
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
singleShot = AcquisitionMode 1
continuousCapture = AcquisitionMode 0

foreign import ccall unsafe "camwire.h camwire_get_single_shot" c_camwire_get_single_shot :: CamwireUnmanagedHandle -> Ptr AcquisitionMode -> IO CamwireResult

foreign import ccall unsafe "camwire.h camwire_set_single_shot" c_camwire_set_single_shot :: CamwireUnmanagedHandle -> AcquisitionMode -> IO CamwireResult

newtype CameraRunning = CameraRunning CInt deriving (Eq,Show,Storable)
cameraRunning = CameraRunning 1
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
    alignment _ = #{alignment struct timespec}
    sizeOf _ = #{size struct timespec}
    peek ptr = do
      s :: CTime <- #{peek struct timespec, tv_sec} ptr
      ns :: CLong <- #{peek struct timespec, tv_nsec} ptr
      return (Timespec { seconds = s, nanoseconds = ns })
    poke ptr ts = do
      #{poke struct timespec, tv_sec} ptr (seconds ts)
      #{poke struct timespec, tv_nsec} ptr (nanoseconds ts)

foreign import ccall unsafe "camwire.h camwire_get_timestamp" c_camwire_get_timestamp :: CamwireUnmanagedHandle -> Ptr Timespec -> IO CamwireResult

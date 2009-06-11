module Paths_hscamwire (
    version,
    getBinDir, getLibDir, getDataDir, getLibexecDir,
    getDataFileName
  ) where

import Data.Version (Version(..))
import System.Environment (getEnv)

version :: Version
version = Version {versionBranch = [0,1], versionTags = []}

bindir, libdir, datadir, libexecdir :: FilePath

bindir     = "/Users/ross/.cabal/bin"
libdir     = "/Users/ross/.cabal/lib/hscamwire-0.1/ghc-6.10.1"
datadir    = "/Users/ross/.cabal/share/hscamwire-0.1"
libexecdir = "/Users/ross/.cabal/libexec"

getBinDir, getLibDir, getDataDir, getLibexecDir :: IO FilePath
getBinDir = catch (getEnv "hscamwire_bindir") (\_ -> return bindir)
getLibDir = catch (getEnv "hscamwire_libdir") (\_ -> return libdir)
getDataDir = catch (getEnv "hscamwire_datadir") (\_ -> return datadir)
getLibexecDir = catch (getEnv "hscamwire_libexecdir") (\_ -> return libexecdir)

getDataFileName :: FilePath -> IO FilePath
getDataFileName name = do
  dir <- getDataDir
  return (dir ++ "/" ++ name)

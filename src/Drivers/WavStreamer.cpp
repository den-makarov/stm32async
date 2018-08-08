/*******************************************************************************
 * StmPlusPlus: object-oriented library implementing device drivers for 
 * STM32F3 and STM32F4 MCU
 * *****************************************************************************
 * Copyright (C) 2016-2017 Mikhail Kulesh
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************/

#include "WavStreamer.h"
#include "../UsartLogger.h"

#ifdef HAL_SD_MODULE_ENABLED
#ifdef HAL_I2S_MODULE_ENABLED

using namespace Stm32async::Drivers;

#define USART_DEBUG_MODULE "WAV: "

/************************************************************************
 * Class WavStreamer
 ************************************************************************/

#define WAV_HEADER_LENGTH sizeof(wavHeader)

WavStreamer::WavStreamer (SdCardFat & _sdCard, AudioDac_UDA1334 & _audioDac) :
    handler { NULL },
    audioDac { _audioDac },
    sdCard { _sdCard },
    sdCardBlock {  },
    wavHeader {  },
    totalBytes { 0 },
    totalBytesRead { 0 },
    volume { 1.0 }
{
    // empty
}

bool WavStreamer::start (AudioDac_UDA1334::SourceType s, const char * fileName)
{
    if (handler != NULL)
    {
        if (!handler->onStartSteaming(s))
        {
            return false;
        }
    }
    
    uint32_t standard, audioFreq;
    if (s == AudioDac_UDA1334::SourceType::STREAM)
    {
        if (!startSdCard(fileName))
        {
            return false;
        }
        standard = I2S_STANDARD_PHILIPS; //I2S_STANDARD_PCM_SHORT;
        audioFreq = wavHeader.fields.samplesPerSec;
    }
    else
    {
        standard = I2S_STANDARD_PHILIPS;
        audioFreq = I2S_AUDIOFREQ_96K;
    }
    
    return audioDac.start(s, standard, audioFreq, I2S_DATAFORMAT_16B);
}

void WavStreamer::stop ()
{
    audioDac.stop();
    totalBytes = totalBytesRead = 0;
    USART_DEBUG("WAV streaming stopped." << UsartLogger::ENDL);
    if (handler != NULL)
    {
        handler->onFinishSteaming();
    }
}

void WavStreamer::periodic ()
{
    if (!audioDac.isActive())
    {
        return;
    }
    if (audioDac.getSourceType() == AudioDac_UDA1334::SourceType::STREAM)
    {
        if (!sdCard.isCardInserted())
        {
            stop();
        }
        if (audioDac.isBlockRequested())
        {
            if (totalBytesRead >= totalBytes)
            {
                stop();
            }
            else
            {
                readBlock();
                audioDac.confirmBlock();
            }
        }
    }
}

void WavStreamer::readBlock ()
{
    UINT bytesRead = 0;
    uint8_t * ptr = &(sdCardBlock.bytes[0]);
    FRESULT code = f_read(&wavFile, ptr, BLOCK_SIZE, &bytesRead);
    totalBytesRead += bytesRead;
    if (code != FR_OK)
    {
        USART_DEBUG("Can not read next block: err=" << code << UsartLogger::ENDL);
    }
    else
    {
        uint32_t wordsRead = bytesRead / 2;
        uint32_t blockSize = std::min(wordsRead, audioDac.getBlockSize());
        uint16_t * block = audioDac.getBlockPtr();
        for (size_t i = 0; i < blockSize; ++i)
        {
            block[i] = (int16_t) (volume * (int16_t) sdCardBlock.words[i]);
        }
        if (blockSize == 0)
        {
            USART_DEBUG("Last block processed: totalBytesRead=" << totalBytesRead << ", totalBytes=" << totalBytes << UsartLogger::ENDL);
            totalBytesRead = totalBytes;
        }
        if (blockSize != audioDac.getBlockSize())
        {
            for (size_t i = blockSize; i < audioDac.getBlockSize(); ++i)
            {
                block[i] = AudioDac_UDA1334::MSB_OFFSET;
            }
        }
    }
}

bool WavStreamer::startSdCard (const char * fileName)
{
    FRESULT code = f_open(&wavFile, fileName, FA_READ);
    if (code != FR_OK)
    {
        USART_DEBUG("Can not open WAV file " << fileName << ": " << code << UsartLogger::ENDL);
        USART_DEBUG("Available files are:" << UsartLogger::ENDL);
        sdCard.listFiles();
        return false;
    }
    
    UINT bytesRead = 0;
    code = f_read(&wavFile, &(sdCardBlock.block[0]), BLOCK_SIZE, &bytesRead);
    if (code != FR_OK || bytesRead != BLOCK_SIZE)
    {
        USART_DEBUG("Can not read WAV header from file " << fileName << ": " << code << UsartLogger::ENDL);
        return false;
    }
    
    for (size_t i = 0; i < WAV_HEADER_LENGTH; ++i)
    {
        wavHeader.header[i] = sdCardBlock.bytes[i];
        sdCardBlock.bytes[i] = 0;
    }
    
    // Check the file type
    if (::strncmp(wavHeader.fields.RIFF, "RIFF", 4) != 0 || ::strncmp(wavHeader.fields.WAVE, "WAVE", 4) != 0)
    {
        USART_DEBUG("File " << fileName << " if not a WAV file" << UsartLogger::ENDL);
        return false;
    }
    
    // Number of bytes per sample
    uint16_t bytesPerSample = wavHeader.fields.numOfChan * wavHeader.fields.bitsPerSample / 8;
    
    // How many samples are in the wav file?
    totalBytes = wavHeader.fields.chunkSize;
    
    if (IS_USART_DEBUG_ACTIVE())
    {
        char riffString[5];
        ::strncpy(riffString, wavHeader.fields.RIFF, 4);
        riffString[4] = 0;
        char waveString[5];
        ::strncpy(waveString, wavHeader.fields.WAVE, 4);
        waveString[4] = 0;
        USART_DEBUG("WAV streaming from file started: " << fileName << UsartLogger::ENDL
                    << UsartLogger::TAB << "RIFF Header = " << riffString << UsartLogger::ENDL
                    << UsartLogger::TAB << "WAVE Header = " << waveString << UsartLogger::ENDL
                    << UsartLogger::TAB << "audioFormat = " << wavHeader.fields.audioFormat << UsartLogger::ENDL
                    << UsartLogger::TAB << "numOfChan = " << wavHeader.fields.numOfChan << UsartLogger::ENDL
                    << UsartLogger::TAB << "samplesPerSec = " << wavHeader.fields.samplesPerSec << UsartLogger::ENDL
                    << UsartLogger::TAB << "bytesPerSec = " << wavHeader.fields.bytesPerSec << UsartLogger::ENDL
                    << UsartLogger::TAB << "blockAlign = " << wavHeader.fields.blockAlign << UsartLogger::ENDL
                    << UsartLogger::TAB << "bitsPerSample = " << wavHeader.fields.bitsPerSample << UsartLogger::ENDL
                    << UsartLogger::TAB << "chunkSize = " << totalBytes << UsartLogger::ENDL
                    << UsartLogger::TAB << "bytesPerSample = " << bytesPerSample << UsartLogger::ENDL
                    << UsartLogger::TAB << "total samples = " << totalBytes / bytesPerSample << UsartLogger::ENDL);
    }
    
    f_lseek(&wavFile, WAV_HEADER_LENGTH);
    totalBytesRead = 0;
    
    return true;
}

#endif
#endif

#pragma once

#include <memory>
#include <algorithm>
#include "can_frame_types.h"
#include "drivers/can_driver.h"
#include "can_helpers.h"
#include "shared_types.h"
#include "log_buffer.h"

#ifndef NATIVE_BUILD
#include <Arduino.h>
#endif

inline LogRingBuffer logRing;

struct CarManagerBase
{
    Shared<int> speedProfile{1};
    Shared<bool> FSDEnabled{false};
    Shared<bool> enablePrint{true};
    Shared<uint32_t> frameCount{0};
    Shared<uint32_t> framesSent{0};
    Shared<int> speedOffset{0};
    virtual void handleMessage(CanFrame &frame, CanDriver &driver) = 0;
    virtual const uint32_t *filterIds() const = 0;
    virtual uint8_t filterIdCount() const = 0;
    virtual ~CarManagerBase() = default;
};

struct LegacyHandler : public CarManagerBase
{
    const uint32_t *filterIds() const override
    {
        static constexpr uint32_t ids[] = {69, 1006};
        return ids;
    }
    uint8_t filterIdCount() const override { return 2; }

    void handleMessage(CanFrame &frame, CanDriver &driver) override
    {
        // STW_ACTN_RQ (0x045 = 69): Follow-Distance-Stalk as Source for Profile Mapping
        // byte[1]: 0x00=Pos1, 0x21=Pos2, 0x42=Pos3, 0x64=Pos4, 0x85=Pos5, 0xA6=Pos6, 0xC8=Pos7
        if (frame.id == 69)
        {
            if (frame.dlc < 2)
                return;
            uint8_t pos = frame.data[1] >> 5;
            if (pos <= 1)
                speedProfile = 2;
            else if (pos == 2)
                speedProfile = 1;
            else
                speedProfile = 0;
            return;
        }
        if (frame.id == 1006)
        {
            if (frame.dlc < 8)
                return;
            auto index = readMuxID(frame);
            if (index == 0)
                FSDEnabled = isFSDSelectedInUI(frame);
            if (index == 0 && FSDEnabled)
            {
                setBit(frame, 46, true);
                setSpeedProfileV12V13(frame, speedProfile);
                framesSent++;
                driver.send(frame);
            }
            if (index == 1)
            {
                setBit(frame, 19, false);
                framesSent++;
                driver.send(frame);
            }
            if (index == 0 && enablePrint)
            {
                char buf[LogRingBuffer::kMaxMsgLen];
                snprintf(buf, sizeof(buf), "LegacyHandler: FSD: %d, Profile: %d",
                         (bool)FSDEnabled, (int)speedProfile);
                logRing.push(buf,
#ifndef NATIVE_BUILD
                             millis()
#else
                             0
#endif
                );
#ifndef NATIVE_BUILD
                Serial.println(buf);
#endif
            }
        }
    }
};

struct HW3Handler : public CarManagerBase
{
    const uint32_t *filterIds() const override
    {
        static constexpr uint32_t ids[] = {1016, 1021};
        return ids;
    }
    uint8_t filterIdCount() const override { return 2; }

    void handleMessage(CanFrame &frame, CanDriver &driver) override
    {
        if (frame.id == 1016)
        {
            if (frame.dlc < 6)
                return;
            uint8_t followDistance = (frame.data[5] & 0b11100000) >> 5;
            switch (followDistance)
            {
            case 1:
                speedProfile = 2;
                break;
            case 2:
                speedProfile = 1;
                break;
            case 3:
                speedProfile = 0;
                break;
            default:
                break;
            }
            return;
        }
        if (frame.id == 1021)
        {
            if (frame.dlc < 8)
                return;
            auto index = readMuxID(frame);
            if (index == 0)
                FSDEnabled = isFSDSelectedInUI(frame);
            if (index == 0 && FSDEnabled)
            {
                speedOffset = std::max(std::min(((uint8_t)((frame.data[3] >> 1) & 0x3F) - 30) * 5, 100), 0);
                setBit(frame, 46, true);
                setSpeedProfileV12V13(frame, speedProfile);
                framesSent++;
                driver.send(frame);
            }
            if (index == 1)
            {
                setBit(frame, 19, false);
                framesSent++;
                driver.send(frame);
            }
            if (index == 2 && FSDEnabled)
            {
                frame.data[0] &= ~(0b11000000);
                frame.data[1] &= ~(0b00111111);
                frame.data[0] |= (speedOffset & 0x03) << 6;
                frame.data[1] |= (speedOffset >> 2);
                framesSent++;
                driver.send(frame);
            }
            if (index == 0 && enablePrint)
            {
                char buf[LogRingBuffer::kMaxMsgLen];
                snprintf(buf, sizeof(buf), "HW3Handler: FSD: %d, Profile: %d, Offset: %d",
                         (bool)FSDEnabled, (int)speedProfile, (int)speedOffset);
                logRing.push(buf,
#ifndef NATIVE_BUILD
                             millis()
#else
                             0
#endif
                );
#ifndef NATIVE_BUILD
                Serial.println(buf);
#endif
            }
        }
    }
};

/**
 * NagHandler — Autosteer nag suppression (counter+1 echo method)
 *
 * Replicates the Chinese TSL6P module behavior:
 * - Listens for CAN 880 (0x370) = EPAS3P_sysStatus
 * - When handsOnLevel = 0 (nag would trigger):
 *   1. Copies the real frame
 *   2. Sets byte 3 = 0xB6 (fixed torsionBarTorque = 1.80 Nm)
 *   3. Sets byte 4 |= 0x40 (handsOnLevel = 1)
 *   4. Increments counter (byte 6 lower nibble + 1)
 *   5. Recalculates checksum (byte 7)
 * - The real EPAS frame with the same counter arrives AFTER -> rejected as duplicate
 *
 * Tested: Model Y Performance 2022 HW3, Basic Autopilot
 * Bus: X179 pin 2/3 (CAN bus 4)
 *
 * Enable with build flag: -D NAG_KILLER
 */
struct NagHandler : public CarManagerBase
{
    Shared<bool> nagKillerActive{true};
    Shared<uint32_t> nagEchoCount{0};

    const uint32_t *filterIds() const override
    {
        static constexpr uint32_t ids[] = {880};
        return ids;
    }
    uint8_t filterIdCount() const override { return 1; }

    void handleMessage(CanFrame &frame, CanDriver &driver) override
    {
        if (frame.id != 880 || frame.dlc < 8)
            return;

        uint8_t handsOn = (frame.data[4] >> 6) & 0x03;

        if (!nagKillerActive || handsOn != 0)
            return;

        CanFrame echo;
        echo.id = 880;
        echo.dlc = 8;

        echo.data[0] = frame.data[0];
        echo.data[1] = frame.data[1];
        echo.data[2] = (frame.data[2] & 0xF0) | 0x08;
        echo.data[5] = frame.data[5];

        // Fixed torque = 1.80 Nm (tRaw = 0x08B6)
        echo.data[3] = 0xB6;

        // handsOnLevel = 1
        echo.data[4] = frame.data[4] | 0x40;

        // Counter + 1
        uint8_t cnt = (frame.data[6] & 0x0F);
        cnt = (cnt + 1) & 0x0F;
        echo.data[6] = (frame.data[6] & 0xF0) | cnt;

        // Checksum: sum(byte0..byte6) + 0x73
        uint16_t sum = echo.data[0] + echo.data[1] + echo.data[2] + echo.data[3] + echo.data[4] + echo.data[5] + echo.data[6];
        echo.data[7] = static_cast<uint8_t>((sum + 0x73) & 0xFF);

        framesSent++;
        nagEchoCount++;
        driver.send(echo);

        if (enablePrint && (nagEchoCount % 500 == 1))
        {
            char buf[LogRingBuffer::kMaxMsgLen];
            snprintf(buf, sizeof(buf), "NagHandler: echo=%u",
                     (unsigned int)(uint32_t)nagEchoCount);
            logRing.push(buf,
#ifndef NATIVE_BUILD
                         millis()
#else
                         0
#endif
            );
#ifndef NATIVE_BUILD
            Serial.println(buf);
#endif
        }
    }
};

struct HW4Handler : public CarManagerBase
{
    const uint32_t *filterIds() const override
    {
#if defined(ISA_SPEED_CHIME_SUPPRESS)
        static constexpr uint32_t ids[] = {921, 1016, 1021};
        return ids;
    }
    uint8_t filterIdCount() const override { return 3; }
#else
        static constexpr uint32_t ids[] = {1016, 1021};
        return ids;
    }
    uint8_t filterIdCount() const override { return 2; }
#endif

    void handleMessage(CanFrame &frame, CanDriver &driver) override
    {
#if defined(ISA_SPEED_CHIME_SUPPRESS)
        if (frame.id == 921)
        {
            if (frame.dlc < 8)
                return;
            if (!isaSpeedChimeSuppressRuntime)
                return;
            frame.data[1] |= 0x20;
            uint8_t sum = 0;
            for (int i = 0; i < 7; i++)
                sum += frame.data[i];
            sum += (921 & 0xFF) + (921 >> 8);
            frame.data[7] = sum & 0xFF;
            framesSent++;
            driver.send(frame);
            return;
        }
#endif
        if (frame.id == 1016)
        {
            if (frame.dlc < 6)
                return;
            auto fd = (frame.data[5] & 0b11100000) >> 5;
            switch (fd)
            {
            case 1:
                speedProfile = 3;
                break;
            case 2:
                speedProfile = 2;
                break;
            case 3:
                speedProfile = 1;
                break;
            case 4:
                speedProfile = 0;
                break;
            case 5:
                speedProfile = 4;
                break;
            }
        }
        if (frame.id == 1021)
        {
            if (frame.dlc < 8)
                return;
            auto index = readMuxID(frame);
            if (index == 0)
                FSDEnabled = isFSDSelectedInUI(frame);
            if (index == 0 && FSDEnabled)
            {
                setBit(frame, 46, true);
                setBit(frame, 60, true);
#if defined(EMERGENCY_VEHICLE_DETECTION)
                if (emergencyVehicleDetectionRuntime)
                    setBit(frame, 59, true);
#endif
                framesSent++;
                driver.send(frame);
            }
            if (index == 1)
            {
                setBit(frame, 19, false);
                setBit(frame, 47, true);
                framesSent++;
                driver.send(frame);
            }
            if (index == 2)
            {
                frame.data[7] &= ~(0x07 << 4);
                frame.data[7] |= (speedProfile & 0x07) << 4;
                framesSent++;
                driver.send(frame);
            }
            if (index == 0 && enablePrint)
            {
                char buf[LogRingBuffer::kMaxMsgLen];
                snprintf(buf, sizeof(buf), "HW4Handler: FSD: %d, Profile: %d",
                         (bool)FSDEnabled, (int)speedProfile);
                logRing.push(buf,
#ifndef NATIVE_BUILD
                             millis()
#else
                             0
#endif
                );
#ifndef NATIVE_BUILD
                Serial.println(buf);
#endif
            }
        }
    }
};

#if defined(SNIFFER)
/**
 * SnifferHandler — read-only mode for frame analysis
 *
 * Listens to specific frames of interest without modifying anything.
 * Useful for dumping frame contents on new/blocked firmware versions.
 *
 * Target frames:
 *   0x3FD (1021) — AP control signals
 *   0x7FF (2047) — Car config
 *   0x3C8  (968) — Country code
 *   0x398  (920) — Alternate car config
 *
 * Enable with: #define SNIFFER in sketch_config.h
 */
struct SnifferHandler : public CarManagerBase
{
    const uint32_t *filterIds() const override
    {
        static constexpr uint32_t ids[] = {1021, 2047, 968, 920};
        return ids;
    }
    uint8_t filterIdCount() const override { return 4; }

    void handleMessage(CanFrame &frame, CanDriver &driver) override
    {
#ifndef NATIVE_BUILD
        Serial.printf("CAN 0x%03X (%4d) DLC:%d  ", frame.id, frame.id, frame.dlc);
        for (int i = 0; i < frame.dlc; i++)
            Serial.printf("%02X ", frame.data[i]);
        Serial.println();
#endif
    }
};
#endif

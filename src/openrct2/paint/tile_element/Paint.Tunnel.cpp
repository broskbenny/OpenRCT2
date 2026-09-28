
#include "Paint.Tunnel.h"

#include "../../core/EnumUtils.hpp"
#include "../Paint.h"

#include <cstdint>

using namespace OpenRCT2;

static constexpr std::array<TunnelDescriptor, kTunnelTypeCount> kTunnelDescriptors = {{
    { 2, 2, 0,   15, TunnelType::standardFlat,                    36 },
    { 3, 3, 0,   15, TunnelType::standardFlat,                    40 },
    { 3, 5, -32,  4, TunnelType::standardFlat,                    44 },
    { 3, 3, 0,   15, TunnelType::invertedFlat,                    48 },
    { 4, 4, 0,   15, TunnelType::invertedFlat,                    52 },
    { 4, 7, -48,  4, TunnelType::invertedFlat,                    56 },
    { 2, 2, 0,   15, TunnelType::squareFlat,                      60 },
    { 3, 3, 0,   15, TunnelType::squareFlat,                      64 },
    { 3, 5, -32,  4, TunnelType::squareFlat,                      68 },
    { 3, 3, 0,   15, TunnelType::squareFlat,                      72 },
    { 2, 3, -16, 15, TunnelType::pathAndMiniGolf,                 76 },
    { 2, 3, -16, 15, TunnelType::path11,                          80 },
    { 2, 3, -16,  4, TunnelType::standardFlatTo25Deg,             36 },
    { 3, 4, -16,  4, TunnelType::invertedFlatTo25Deg,             48 },
    { 2, 3, -16,  4, TunnelType::squareFlatTo25Deg,               60 },
    { 3, 4, -16,  4, TunnelType::squareFlatTo25Deg,               72 },
    { 2, 2, 0,   15, TunnelType::doorClosed,                      84 },
    { 2, 2, 0,   15, TunnelType::doorOpeningOutward,              88 },
    { 2, 2, 0,   15, TunnelType::doorOpenOutward,                 92 },
    { 2, 2, 0,   15, TunnelType::doorOpeningInward,               96 },
    { 2, 2, 0,   15, TunnelType::doorOpenInward,                 100 },
    { 2, 3, -16,  4, TunnelType::doorClosedFlatToDown25,          84 },
    { 2, 3, -16,  4, TunnelType::doorOpeningOutwardFlatToDown25,  88 },
    { 2, 3, -16,  4, TunnelType::doorOpenOutwardFlatToDown25,     92 },
    { 2, 3, -16,  4, TunnelType::doorOpeningInwardFlatToDown25,   96 },
    { 2, 3, -16,  4, TunnelType::doorOpenInwardFlatToDown25,     100 },
}};

const TunnelDescriptor& GetTunnelDescriptor(TunnelType type)
{
    return kTunnelDescriptors[EnumValue(type)];
}

using TunnelGroupMap = std::array<TunnelType, kTunnelSubTypeCount>;
static std::array<TunnelGroupMap, kTunnelGroupCount> tunnelMap = {
    TunnelGroupMap{ TunnelType::standardFlat, TunnelType::standardSlopeStart, TunnelType::standardSlopeEnd,
                    TunnelType::standardFlatTo25Deg, TunnelType::invertedFlat },
    TunnelGroupMap{ TunnelType::squareFlat, TunnelType::squareSlopeStart, TunnelType::squareSlopeEnd,
                    TunnelType::squareFlatTo25Deg, TunnelType::invertedSquare },
    TunnelGroupMap{ TunnelType::invertedFlat, TunnelType::invertedSlopeStart, TunnelType::invertedSlopeEnd,
                    TunnelType::invertedFlatTo25Deg, TunnelType::invertedFlat },
};

void PaintUtilPushTunnelLeft(PaintSession& session, uint16_t height, TunnelType type)
{
    if (!session.LeftTunnels.full())
    {
        session.LeftTunnels.emplace_back(height / kCoordsZPerTinyZ, type);
    }
}

void PaintUtilPushTunnelRight(PaintSession& session, uint16_t height, TunnelType type)
{
    if (!session.RightTunnels.full())
    {
        session.RightTunnels.emplace_back(height / kCoordsZPerTinyZ, type);
    }
}

void PaintUtilSetVerticalTunnel(PaintSession& session, uint16_t height)
{
    session.VerticalTunnelHeight = height / 16;
}

void PaintUtilPushTunnelRotated(PaintSession& session, uint8_t direction, uint16_t height, TunnelType type)
{
    if (direction & 1)
    {
        PaintUtilPushTunnelRight(session, height, type);
    }
    else
    {
        PaintUtilPushTunnelLeft(session, height, type);
    }
}

void TrackPaintUtilRightQuarterTurn5TilesTunnel(
    PaintSession& session, TunnelGroup group, TunnelSubType tunnelType, int16_t height, Direction direction,
    uint8_t trackSequence)
{
    if (direction == 0 && trackSequence == 0)
    {
        PaintUtilPushTunnelLeft(session, height, group, tunnelType);
    }
    if (direction == 0 && trackSequence == 6)
    {
        PaintUtilPushTunnelRight(session, height, group, tunnelType);
    }
    if (direction == 1 && trackSequence == 6)
    {
        PaintUtilPushTunnelLeft(session, height, group, tunnelType);
    }
    if (direction == 3 && trackSequence == 0)
    {
        PaintUtilPushTunnelRight(session, height, group, tunnelType);
    }
}

void TrackPaintUtilRightQuarterTurn3TilesTunnel(
    PaintSession& session, int16_t height, Direction direction, uint8_t trackSequence, TunnelType tunnelType)
{
    if (direction == 0 && trackSequence == 0)
    {
        PaintUtilPushTunnelLeft(session, height, tunnelType);
    }

    if (direction == 0 && trackSequence == 3)
    {
        PaintUtilPushTunnelRight(session, height, tunnelType);
    }

    if (direction == 1 && trackSequence == 3)
    {
        PaintUtilPushTunnelLeft(session, height, tunnelType);
    }

    if (direction == 3 && trackSequence == 0)
    {
        PaintUtilPushTunnelRight(session, height, tunnelType);
    }
}

void TrackPaintUtilRightQuarterTurn3TilesTunnel(
    PaintSession& session, TunnelGroup group, TunnelSubType tunnelType, int16_t height, Direction direction,
    uint8_t trackSequence)
{
    if (direction == 0 && trackSequence == 0)
    {
        PaintUtilPushTunnelLeft(session, height, group, tunnelType);
    }

    if (direction == 0 && trackSequence == 3)
    {
        PaintUtilPushTunnelRight(session, height, group, tunnelType);
    }

    if (direction == 1 && trackSequence == 3)
    {
        PaintUtilPushTunnelLeft(session, height, group, tunnelType);
    }

    if (direction == 3 && trackSequence == 0)
    {
        PaintUtilPushTunnelRight(session, height, group, tunnelType);
    }
}

void TrackPaintUtilRightQuarterTurn3Tiles25DegUpTunnel(
    PaintSession& session, TunnelGroup group, int16_t height, Direction direction, uint8_t trackSequence,
    TunnelSubType tunnelType0, TunnelSubType tunnelType3)
{
    if (direction == 0 && trackSequence == 0)
    {
        PaintUtilPushTunnelLeft(session, height - 8, group, tunnelType0);
    }
    if (direction == 0 && trackSequence == 3)
    {
        PaintUtilPushTunnelRight(session, height + 8, group, tunnelType3);
    }
    if (direction == 1 && trackSequence == 3)
    {
        PaintUtilPushTunnelLeft(session, height + 8, group, tunnelType3);
    }
    if (direction == 3 && trackSequence == 0)
    {
        PaintUtilPushTunnelRight(session, height - 8, group, tunnelType0);
    }
}

void TrackPaintUtilRightQuarterTurn3Tiles25DegDownTunnel(
    PaintSession& session, TunnelGroup group, int16_t height, Direction direction, uint8_t trackSequence,
    TunnelSubType tunnelType0, TunnelSubType tunnelType3)
{
    if (direction == 0 && trackSequence == 0)
    {
        PaintUtilPushTunnelLeft(session, height + 8, group, tunnelType0);
    }
    if (direction == 0 && trackSequence == 3)
    {
        PaintUtilPushTunnelRight(session, height - 8, group, tunnelType3);
    }
    if (direction == 1 && trackSequence == 3)
    {
        PaintUtilPushTunnelLeft(session, height - 8, group, tunnelType3);
    }
    if (direction == 3 && trackSequence == 0)
    {
        PaintUtilPushTunnelRight(session, height + 8, group, tunnelType0);
    }
}

void TrackPaintUtilLeftQuarterTurn3TilesTunnel(
    PaintSession& session, TunnelGroup group, TunnelSubType tunnelType, int16_t height, Direction direction,
    uint8_t trackSequence)
{
    if (direction == 0 && trackSequence == 0)
    {
        PaintUtilPushTunnelLeft(session, height, group, tunnelType);
    }

    if (direction == 2 && trackSequence == 3)
    {
        PaintUtilPushTunnelRight(session, height, group, tunnelType);
    }

    if (direction == 3 && trackSequence == 0)
    {
        PaintUtilPushTunnelRight(session, height, group, tunnelType);
    }

    if (direction == 3 && trackSequence == 3)
    {
        PaintUtilPushTunnelLeft(session, height, group, tunnelType);
    }
}

void TrackPaintUtilRightQuarterTurn1TileTunnel(
    PaintSession& session, TunnelGroup group, Direction direction, uint16_t baseHeight, int8_t startOffset,
    TunnelSubType startTunnel, int8_t endOffset, TunnelSubType endTunnel)
{
    TrackPaintUtilLeftQuarterTurn1TileTunnel(
        session, group, DirectionPrev(direction), baseHeight, endOffset, endTunnel, startOffset, startTunnel);
}

void TrackPaintUtilLeftQuarterTurn1TileTunnel(
    PaintSession& session, Direction direction, uint16_t baseHeight, int8_t startOffset, TunnelType startTunnel,
    int8_t endOffset, TunnelType endTunnel)
{
    switch (direction)
    {
        case 0:
            PaintUtilPushTunnelLeft(session, baseHeight + startOffset, startTunnel);
            break;
        case 2:
            PaintUtilPushTunnelRight(session, baseHeight + endOffset, endTunnel);
            break;
        case 3:
            PaintUtilPushTunnelRight(session, baseHeight + startOffset, startTunnel);
            PaintUtilPushTunnelLeft(session, baseHeight + endOffset, endTunnel);
            break;
    }
}

void TrackPaintUtilLeftQuarterTurn1TileTunnel(
    PaintSession& session, TunnelGroup group, Direction direction, uint16_t baseHeight, int8_t startOffset,
    TunnelSubType startTunnel, int8_t endOffset, TunnelSubType endTunnel)
{
    switch (direction)
    {
        case 0:
            PaintUtilPushTunnelLeft(session, baseHeight + startOffset, group, startTunnel);
            break;
        case 2:
            PaintUtilPushTunnelRight(session, baseHeight + endOffset, group, endTunnel);
            break;
        case 3:
            PaintUtilPushTunnelRight(session, baseHeight + startOffset, group, startTunnel);
            PaintUtilPushTunnelLeft(session, baseHeight + endOffset, group, endTunnel);
            break;
    }
}

TunnelType GetTunnelType(TunnelGroup tunnelGroup, TunnelSubType tunnelSubType)
{
    return tunnelMap[EnumValue(tunnelGroup)][EnumValue(tunnelSubType)];
}

// CCam::Process_FollowPed_SA(CVector const&, float, float, float, bool)
// ARM Thumb2 reverse - GTA SA Mobile (Android)
// Register mapping at entry:
//   R0  = this (CCam*)   → saved to R10
//   R1  = &TargetCoors   → saved to R5
//   R2  = fTargetOrient  (float, 1st stack-after-regs arg in legacy ABI)
//   R3  = fStickX
//   [SP+0] = fStickY
//   [SP+4] = bResetCam   → loaded via [R7+0xC] as R4

// External symbols referenced (abbreviated)
//   FindPlayerPed(int)           → BLX 0x19F8F4
//   sinf(float as int)           → BLX 0x18C778
//   cosf(float as int)           → BLX 0x18F664
//   atan2f(EOR-negX, Y)          → BLX 0x18D51C
//   acosf(float as int)          → BLX 0x19AA70
//   GetVehicle(CVehicle*)        → BLX 0x18D34C
//   GetVehiclePassenger          → BLX 0x18D400
//   IsOnBike                     → BLX 0x18A39C
//   HasVehicleBackView           → BLX 0x18CC90
//   NormaliseVector3D            → BLX 0x19BA04
//   GetAngleBetweenVectors       → BLX 0x1983CC
//   LimitAngle(float)            → BLX 0x18F214 (returns 1 if behind)
//   sub_19C928                   → smooth/lerp helper
//   sub_19159C                   → atan-like helper used for atanf-clamp
//   sub_18B818                   → cross product
//   sub_18AE74                   → ProcessColModel (cam collision)
//   sub_199F34                   → CCamPathSplines (or CCam::Process bullet?)
//   sub_192378                   → GetPadStickXState or GetPadMoveStickInput
//   sub_1906A0                   → IsAimButtonPressed
//   sub_193054                   → GetRopeOrMount on ped
//   sub_194010                   → UpdateCameraXYPosition_Touch
//   sub_194D9C                   → UpdateCameraXYPosition_Mouse
//   sub_19000C                   → GetEnteringVehicle
//   sub_18E310                   → IsPlayerOnMovingGround / facing dir
//   CTimer::ms_fTimeStep         → float*
//   CTimer::m_snTimeInMilliseconds → uint32_t*
//   CGame::currArea              → int*
//   TheCamera                    → CCamera*
//   PEDCAM_SET                   → float[]  (array of cam-setting floats)
//   gLastCamDist                 → float**
//   gForceCamBehindPlayer        → uint8_t**
//   AIMWEAPON_STICK_SENS         → float*
//   dword_952D58/64              → global float[3] used as reset-cam target pos cache
//   dword_6AA08C / _090          → global floats (smoothed heading + pitch targets)

void CCam::Process_FollowPed_SA(
    const CVector& TargetCoors,
    float /*fTargetOrient*/,
    float /*fStickX*/,
    float /*fStickY*/,
    bool  bResetCam)
{
    // ── 1. Early validation ────────────────────────────────────────────────
    CPed* pPed = (CPed*)m_pTargetEntity;                 // [this+0x1F4]

    // Entity must be of ped-class (nType & 7 == 3)
    if ((*(uint8_t*)((char*)pPed + 0x3A) & 7) != 3)
        return;

    // Additional check (e.g. is player controlled / not dead)
    if (sub_198B44() != 1)
        return;

    // ── 2. Grab ped / vehicle references ──────────────────────────────────
    CPed*     pPlayerPed = FindPlayerPed(0);             // R6
    CPed*     pTargetPed = (CPed*)m_pTargetEntity;       // R8 — kept for the whole function
    int       nPedSubType = *(int*)((char*)pTargetPed + 0x59C);
    CVehicle* pVehicle    = (CVehicle*)*(uintptr_t*)((char*)pTargetPed + 0x440);

    if (nPedSubType == 1)
        pPlayerPed = FindPlayerPed(1);

    // ── 3. Cache target coordinates & area-based settings index ───────────
    //       TargetCoors stored at SP+0x70..0x78
    float fTgtX = TargetCoors.x;
    float fTgtY = TargetCoors.y;
    float fTgtZ = TargetCoors.z;

    // RSB R11, R2, R2 LSL#4  →  R11 = 15 * bInterior
    // used to index into PEDCAM_SET (exterior = [0], interior = [15])
    bool bInInterior = (*CGame::currArea != 0);
    int  nSetOfs     = bInInterior ? 15 : 0;            // R11 (byte offset = nSetOfs*4)

    float* pCS       = PEDCAM_SET + nSetOfs;             // pointer into settings array

    // ── 4. Read initial follow-distance from settings ──────────────────────
    float fFollowDist  = pCS[1];                         // S16  [pCS+4]
    int   nCamMode     = *(int*)((char*)TheCamera + 0xC4);

    // If aiming-mode (3) and ped is swimming, optionally double distance
    if (nCamMode == 3) {
        if (*(uint32_t*)((char*)pTargetPed + 0x484) & 1) {
            if (IsOnBike_sub18A39C(pVehicle)) {
                float fDouble = fFollowDist + fFollowDist;   // VADD S0, S16, S16
                // Only update if result != 0 (IT NE: VMOVNE S16, S0)
                if (fDouble != 0.0f)
                    fFollowDist = fDouble;
            }
        }
    }

    // pointer to flags byte at pTargetPed+0x484, kept for later checks
    uint8_t* pPedFlagsAt484 = (uint8_t*)((char*)pTargetPed + 0x484);

    // Add stored camera distance offset (TheCamera+0xD0)
    fFollowDist += *(float*)((char*)TheCamera + 0xD0);   // VADD S16, S16, S0

    // ── 5. Smooth against last-frame stored distance ───────────────────────
    float& fLastCamDist = *(*gLastCamDist_ptr);
    float  fPrevDist    = fLastCamDist;                  // VLDR S0, [gLastCamDist]

    // Cam-set fields used throughout
    float fMaxFollowDist = pCS[0x38 / 4];               // S2  [pCS+0x38]  stored → SP+0x34
    float fMinHeightAng  = pCS[0x08 / 4];               // S21 [pCS+0x08]  (height angle lower bound)
    float fHeightAng     = pCS[0x0C / 4];               // S26 [pCS+0x0C]  (initial height angle)
    float fFOVMin        = pCS[0x10 / 4];               // S18 [pCS+0x10]  (min follow distance / FOV low)
    float fFOVMax        = pCS[0x34 / 4];               // S20 [pCS+0x34]  stored → SP+0x38

    // If current dist > prev stored, snap height-angle to current dist
    if (fFollowDist > fPrevDist)
        fHeightAng = fFollowDist;                        // VMOVGT S26, S16

    fLastCamDist = fFollowDist;                          // VSTR S16, [gLastCamDist]

    // Store PEDCAM_SET entry pointer and fMaxFollowDist onto stack
    // (used at SP+0x30 and SP+0x34 respectively)

    // ── 6. Camera-mode angle offset ────────────────────────────────────────
    //   Adds a per-mode Z offset to fMinHeightAng
    float fModeAngleAdd = 0.0f;
    if (nCamMode == 3) {
        // aiming mode — additional check on ped flag
        if (*(uint32_t*)((char*)pTargetPed + 0x484) & 1) {
            // double fol dist path already handled; may also swap S16 here
        }
    } else if (nCamMode == 2) {
        int currArea = *CGame::currArea;
        fModeAngleAdd = (currArea == 0)
                        ? *(float*)((char*)this + 0xC4)  // exterior
                        : *(float*)((char*)this + 0xC8); // interior
    } else if (nCamMode == 1) {
        fModeAngleAdd = *(float*)((char*)this + 0xC0);
    }
    fMinHeightAng += fModeAngleAdd;                      // VADD S21, S21, S0

// ── 7. Vehicle-in-vehicle checks; set S23 (heading weight) & S28 (speed weight) ──
    float S23 = 0.0f, S28 = 0.0f;    // default for ped-not-in-vehicle path

    void* pVehPtr = GetVehicle_sub18D34C(pVehicle);

    if (pVehPtr) {
        // Ped IS riding something
        uint16_t nModelA = *(uint16_t*)((char*)pVehPtr + 0xA);
        pVehPtr = GetVehicle_sub18D34C(pVehicle); // re-get parent
        uint16_t nModelB = *(uint16_t*)((char*)pVehPtr + 0xA);

        S23 = 0.5f;
        S28 = 1.0f;
        if (nModelB == 4)   S23 = 0.0f;   // BIKE / QUAD type
        if (nModelA == 0)   S28 = 0.0f;
        // falls through to loc_3C2ED0
    } else {
        // Not in vehicle — check passenger slot
        void* pPassVeh = GetVehiclePassenger_sub18D400(pVehicle);
        S28 = 0.0f;

        if (!pPassVeh) {
            // loc_3C36B0: no vehicle at all — S23 = S28 = 0; loop to loc_3C2ED0
            S23 = 0.0f;
            goto loc_3C2ED0;
        }

        if (*(pPedFlagsAt484) & 1) {
            // ped-flag set → use default S23/S28 = 0
            goto loc_3C2ED0;
        }

        // Ped is in water / swimming
        float fSwimAngSpeed = sub_1A10F0(pPlayerPed);  // get angular speed
        S23 = 3.0f;
        S28 = 0.5f;
        if (fSwimAngSpeed != 0.0f) {
            S23 = 0.0f;
            S28 = 0.0f;
        }
    }

// ── 8. Idle timer + cam-behind flag ────────────────────────────────────
loc_3C2ED0:
    {
        uint8_t bCamFollowActive = *(uint8_t*)((char*)this + 0xA);
        // byte_951FFE relative to TheCamera base
        uint8_t bCamBehindForced = *(uint8_t*)((char*)TheCamera + (0x951FFE - 0x951FA8));

        if (bCamBehindForced)
            goto loc_3C2F5E;  // skip idle logic, already have R0 = 1

        if (!bCamFollowActive) {
            // ── idle timer accumulation ──
            float fIdleTimer = *(float*)((char*)this + 0x8C);
            float fTimeStep  = *CTimer::ms_fTimeStep;
            float fNewTimer  = fTimeStep + fIdleTimer;

            if (fNewTimer >= 70.0f) {
                // Cap: once at/over threshold, subtract instead and clamp to ≥70
                fNewTimer = fIdleTimer - fTimeStep;
                if (fNewTimer < 70.0f) fNewTimer = 70.0f; // VMAXGE D0,D0,D2
            }
            *(float*)((char*)this + 0x8C) = fNewTimer;
            // R0 = 0 → will skip "force behind" path below
            goto loc_3C2F5E;
        }

        // Ped is moving → reset idle timer to 70 (set cam-follow)
        *(float*)((char*)this + 0x8C) = 70.0f; // 0x428C0000
        // R0 = 1
        goto loc_3C2F5E;
    }

// ── 9. Clamp follow dist from below, compute cam Z, check reset ────────
loc_3C2F5E:
    // VMAX D16, D8, D9  →  fFollowDist = max(fFollowDist, fFOVMin)
    fFollowDist = (fFollowDist > fFOVMin) ? fFollowDist : fFOVMin;

    // R4 = bResetCam  (loaded from [R7+0xC] = original SP+4)
    // S24 = pCS[0] + fTgtZ   (pCS entry 0 = base height offset)
    float fCamPosZ = pCS[0] + fTgtZ;     // S24; stored to SP+0x78

    // R0 == 0 means: no vehicle and nothing special → go to bResetCam path
    // (The branch is BEQ.W loc_3C30A8 if R0 == 0)
    if (/* R0 == 0 from idle-timer path */ false)
        goto loc_3C30A8;

// ── 10. bResetCam == 1: snap target pos from ped's matrix ─────────────
loc_3C2F8A:
    if (bResetCam == 1) {
        int pMatrix = *(int*)((char*)pTargetPed + 0x14);
        float* pBase;
        if (pMatrix != 0)
            pBase = (float*)((char*)pMatrix + 0x30);   // matrix 4th row (position)
        else
            pBase = (float*)((char*)pTargetPed + 4);   // fallback to base ped pos

        float rX = pBase[0], rY = pBase[1];
        float rZ = *(float*)((char*)pBase + 8);        // [pBase+8]

        *(float*)(dword_952D58 + 0) = rX;
        *(float*)(dword_952D58 + 4) = rY;
        *(float*)(dword_952D58 + 8) = rZ;
        *(float*)(dword_952D64 + 0) = 0.0f;
        *(float*)(dword_952D64 + 4) = 0.0f;
        *(float*)(dword_952D64 + 8) = 0.0f;

        fTgtX    = rX;
        fTgtY    = rY;
        fTgtZ    = rZ;
        fCamPosZ = pCS[0] + rZ;
    }

// ── 11. Clear cam heading influence; optional vehicle speed contribution ─
// loc_3C2FDA:
    {
        // dword_952B84/88 relative to TheCamera+0x951FA8
        *(float*)((char*)TheCamera + (0x952B84 - 0x951FA8)) = 0.0f;
        *(float*)((char*)TheCamera + (0x952B88 - 0x951FA8)) = 0.0f;

        pVehicle = (CVehicle*)*(uintptr_t*)((char*)pTargetPed + 0x440);

        // Check if vehicle exists and supports "look-behind"
        CVehicle* pVeh2 = (CVehicle*)HasVehicleBackView_sub18CC90(pVehicle, 1);
        if (pVeh2) {
            // pPedFlagsAt484[-1+3] = flags at pTargetPed+0x487 ? → bit 3
            // LDRB R1,[R1,#3]; LSLS #0x1D → bit(3) = 0x08 mask; BPL means bit clear → skip
            if (*(uint8_t*)(pPedFlagsAt484 + 3) & 0x08) {
                if (*(uint8_t*)((char*)pVeh2 + 0x19) == 0) {
                    // Compute velocity magnitude²
                    float vx = *(float*)((char*)pTargetPed + 0x48);
                    float vy = *(float*)((char*)pTargetPed + 0x4C);
                    float vz = *(float*)((char*)pTargetPed + 0x50);
                    float fSpeedSq = vx*vx + vy*vy + vz*vz;

                    // Store -0.35 at TheCamera+0xBE0 (as float bits 0xBEB33333)
                    *(uint32_t*)((char*)TheCamera + (0x952B88 - 0x951FA8)) = 0xBEB33333;

                    // Use lookup table for heading offset: -0.4 or -0.2 depending on speed
                    // dword_3C3C94 = {0xBF333333 (≈-0.7), 0xBE4CCCCC (≈-0.2)}
                    float* pTbl = (float*)dword_3C3C94;
                    float fHdgOfs = pTbl[0];             // default -0.7
                    if (fSpeedSq > 0.000001f)
                        fHdgOfs = pTbl[1];               // -0.2 if moving

                    *(float*)((char*)TheCamera + (0x952B88 - 0x951FA8 + /*BDC*/ 0)) = fHdgOfs;
                }
            }
        }
    }

// ── 12. Reset gForceCamBehindPlayer, set cam flags ─────────────────────
// loc_3C305C:
    *(*gForceCamBehindPlayer_ptr) = 0;
    *(uint8_t*)((char*)this + 0xC) = 0;   // m_bCamDirectlyInFront = false
    *(uint8_t*)((char*)this + 3)   = 1;   // m_bUseMouse3rdPerson = true (or similar)

    {
        // Check if cam is locked (some camera flag)
        uint8_t bCamLocked = *(uint8_t*)((char*)TheCamera + (0x951FCE - 0x951FA8));
        // ORRS.W R8, bCamLocked, bResetCam  → if either non-zero, skip heading calc
        if ((bCamLocked ? 1 : 0) | (int)bResetCam)
            goto loc_3C3240;

        // ── 13. Compute target heading from entity's matrix ──────────────
        int pMat = *(int*)((char*)pTargetPed + 0x14);
        if (pMat == 0)
            goto loc_3C3216;

        {
            // atan2f(negate forward.x, forward.y) to get heading
            // [pMat+0x10] = forward.x (local right in some conventions)
            // [pMat+0x14] = forward.y
            // LDRD.W R2, R1, [R0,#0x10]  →  R2 = *(int*)[pMat+0x10], R1 = *(int*)[pMat+0x14]
            // EOR.W R0, R2, #0x80000000  →  flip sign of x component
            // BLX atan2f(R0=negX_bits, R1=Y_bits)
            int   iX  = *(int*)((char*)pMat + 0x10);
            int   iY  = *(int*)((char*)pMat + 0x14);
            float fHd = atan2f_sub18D51C(iX ^ 0x80000000, iY);  // atan2(-fwdX, fwdY)
            *(float*)((char*)this + 0x94) = fHd - 1.5708f;      // subtract π/2
            goto loc_3C3226_mirrored;
        }

loc_3C3216:
        {
            // No matrix → use a stored heading value
            float fHd = *(float*)((char*)pMat + 0x10);          // fallback
            *(float*)((char*)this + 0x94) = fHd - 1.5708f;
        }

loc_3C3226_mirrored:
        // If camera is mirrored (reverse-view), add π
        if (*(uint8_t*)((char*)TheCamera + (0x951FC3 - 0x951FA8))) {
            *(float*)((char*)this + 0x94) += 3.1416f;
        }
    }

// ── 14. Clear secondary angle fields, check distance changed ───────────
loc_3C3240:
    *(float*)((char*)this + 0x88) = 0.0f;
    *(float*)((char*)this + 0x98) = 0.0f;
    *(float*)((char*)this + 0x9C) = 1000.0f;   // 0x447A0000

    {
        // If follow distance equals cam's stored base dist, update smooth dist
        float fDistCC8 = *(float*)((char*)TheCamera + 0xC8);
        if (fFollowDist == fDistCC8) {
            float fDistCC = *(float*)((char*)TheCamera + 0xCC);
            *(float*)((char*)TheCamera + 0xD0) = fDistCC;
        }
    }

// ── 15. Heading (alpha) and pitch (beta) angles ────────────────────────
    float fAlpha;   // horizontal / yaw    stored at this+0x94
    float fBeta;    // vertical   / pitch  working register (S18 → this+0x84)

    if (bResetCam) {
        // CMP.W R8, #1 (R8 was restored to original pTargetPed ptr) — actually this
        // compares the reset flag.  On reset: use previously stored beta directly.
        fBeta = *(float*)((char*)this + 0x84);
        goto loc_3C3336;
    }

    // Not a reset frame
    *(float*)((char*)this + 0x84) = 0.0f;

    if (!(*(uint8_t*)pPedFlagsAt484 & 1)) {
        fBeta = 0.0f;
        goto loc_3C3336;
    }

    // Compute vertical angle from ped's look-direction dot product
    {
        // Get ped's world velocity direction at offsets 0x57C, 0x580, 0x584
        float fVelFwd  = *(float*)((char*)pTargetPed + 0x580);
        float fVelRight= *(float*)((char*)pTargetPed + 0x57C);
        float fVelUp   = *(float*)((char*)pTargetPed + 0x584);

        int pMat2 = *(int*)((char*)pTargetPed + 0x14);
        // Matrix rows: [pMat+0x10]=fwd.x, [pMat+0x14]=fwd.y, [pMat+0x18]=fwd.z
        float fMX = *(float*)((char*)pMat2 + 0x14);
        float fMY = *(float*)((char*)pMat2 + 0x10);
        float fMZ = *(float*)((char*)pMat2 + 0x18);

        float fDot = fVelFwd*fMX + fVelRight*fMY + fVelUp*fMZ;
        fDot = (fDot >  1.0f) ?  1.0f : fDot;
        fDot = (fDot < -1.0f) ? -1.0f : fDot;

        // VSUB S18, #0, acosf(fDot)  →  fBeta = -acos(dot)
        fBeta = -(acosf_sub19AA70(fDot));
        *(float*)((char*)this + 0x84) = fBeta;
    }

// ── 16. Build camera position from spherical coordinates ──────────────
loc_3C3336:
    fAlpha = *(float*)((char*)this + 0x94);

    {
        float fSinA = sinf_sub18C778(fAlpha);            // R11
        float fSinB = sinf_sub18C778(fBeta);             // R6
        float fCosA = cosf_sub18F664(fAlpha);            // S0 → used directly
        float fCosB = cosf_sub18F664(fBeta);             // S0 (last cosf call)

        // Trig products  (S2=sinB, S4=sinA)
        float sinBcosA =  fSinB * fCosA;                 // S18
        float sinAsinB =  fSinA * fSinB;                 // S20
        // negated versions stored as camera front direction components
        float nsinBcosA = -sinBcosA;                     // S22 → [this+0x16C]
        float nsinAsinB = -sinAsinB;                     // S30 → [this+0x168]

        // fCamDist = max(fFollowDist, fFOVMin)  from S6 = D3[0] loaded off SP+0x40
        // (was saved there by VSTR D16, [SP,#0x40] after VMAX)
        float fCamDist = fFollowDist;   // simplified; above VMAX already applied

        // Store orientation components of camera's "up toward target" vector
        *(float*)((char*)this + 0x168) = nsinAsinB;      // VSTR S30
        *(float*)((char*)this + 0x16C) = nsinBcosA;      // VSTR S22
        *(int  *)((char*)this + 0x170) = *(int*)&fCosB;  // STR R0 (cosBeta bits)

        // Source = TargetCoors + rotation(fCamDist)  (near distance)
        *(float*)((char*)this + 0x1B0) = fTgtX + fCamDist * sinAsinB;
        *(float*)((char*)this + 0x1B4) = fTgtY + fCamDist * sinBcosA;
        *(float*)((char*)this + 0x1B8) = fCamPosZ - fCamDist * fCosB;

        // "Front" target (far distance = fFollowDist)
        *(float*)((char*)this + 0x1BC) = fTgtX + fFollowDist * sinAsinB;
        *(float*)((char*)this + 0x1C0) = fTgtY + fFollowDist * sinBcosA;
        *(float*)((char*)this + 0x1C4) = fCamPosZ - fFollowDist * fCosB;

        // Timestamp
        *(uint32_t*)((char*)this + 0x1E0) = *(*CTimer_ms_snTimeInMilliseconds_ptr);
        *(uint32_t*)((char*)this + 0x1F0) = 0;

        // If not resetting cam (R8==0 path): negate fMinHeightAng into beta
        if (/* R8 == 0 */ !bResetCam) {
            *(float*)((char*)this + 0x84) = -fMinHeightAng;  // VNEGEQ/VSTREQ
        }
    }

// ── 17. Vehicle-on-horse ped: snap source closer ──────────────────────
    // (loc_3C3400 area: called back from force-behind check)
    {
        CVehicle* pVehR4 = (CVehicle*)*(uintptr_t*)((char*)pTargetPed + 0x440);
        if (GetVehicle_sub18D34C(pVehR4)) {
            pVehR4 = (CVehicle*)GetVehicle_sub18D34C(pVehR4);
            uint16_t nModel = *(uint16_t*)((char*)pVehR4 + 0xA);
            if (nModel == 4) {
                // On a bike/horse: offset head angle by a fixed amount
                float fAngOfs = GetVehiclePassenger_sub18D400(pVehR4)
                                ? -0.34907f   // VLDR S0, =-0.34907
                                : -0.2618f;   // VLDR S0, =-0.2618
                *(float*)((char*)this + 0x84) += fAngOfs;
            }
        }
    }

// ── 18. Heading smoothing & direction tracking ────────────────────────
    // The section from loc_3C3448 computes the heading FROM
    // target (alpha) as atan2 of (TargetCoors – Source), then
    // wraps it into [−π, π], checks gForceCamBehindPlayer, and
    // ultimately updates m_fAlpha with stick input.
    {
        // Computed target heading = atan2(-source.x, source.y) − π/2
        float fSrcX = *(float*)((char*)this + 0x168);
        float fSrcY = *(float*)((char*)this + 0x16C);
        float fSrcZ = *(float*)((char*)this + 0x170);

        // Direction from target to camera
        float dX = fTgtX - *(float*)((char*)this + 0x1B0);
        float dY = fTgtY - *(float*)((char*)this + 0x1B4);
        float dZ = fCamPosZ - *(float*)((char*)this + 0x1B8);

        *(float*)((char*)this + 0x168) = dX;
        *(float*)((char*)this + 0x16C) = dY;
        *(float*)((char*)this + 0x170) = dZ;

        NormaliseVector3D_sub19BA04((CVector*)((char*)this + 0x168));

        // Similarly for the "front" direction
        float dX2 = fTgtX - *(float*)((char*)this + 0x1BC);
        float dY2 = fTgtY - *(float*)((char*)this + 0x1C0);
        float dZ2 = fCamPosZ - *(float*)((char*)this + 0x1C4);
        float fDist2 = sqrtf(dX2*dX2 + dY2*dY2 + dZ2*dZ2);

        // Compute new target alpha from direction vector (atan2 of negated X, Y)
        float fNewAlpha = atan2f_sub18D51C(
            *(int*)((char*)this + 0x168) ^ 0x80000000,  // negate x
            *(int*)((char*)this + 0x16C)
        );
        float fNewAlpha2 = fNewAlpha - 1.5708f;         // −π/2 offset

        // Wrap fNewAlpha2 into (−π, π] range
        if (fNewAlpha2 < -3.1416f)     fNewAlpha2 += 6.2832f;
        else if (fNewAlpha2 > 3.1416f) fNewAlpha2 -= 6.2832f;

        // Compute second target alpha from ped matrix (similar atan2 calculation)
        float fPedAlpha;
        int pM = *(int*)((char*)pTargetPed + 0x14);
        if (pM) {
            int iX = *(int*)((char*)pM + 0x10) ^ 0x80000000;
            int iY = *(int*)((char*)pM + 0x14);
            fPedAlpha = atan2f_sub18D51C(iX, iY) - 1.5708f;
        } else {
            fPedAlpha = *(float*)((char*)pTargetPed + 0x10) - 1.5708f;
        }

        // Wrap difference
        float fAlphaDiff = fPedAlpha - fNewAlpha2;
        if (fAlphaDiff >  3.1416f) fAlphaDiff -= 6.2832f;
        if (fAlphaDiff < -3.1416f) fAlphaDiff += 6.2832f;

        // ── gForceCamBehindPlayer logic ─────────────────────────────
        //    If LimitAngle returns 1 → force-behind active
        if (LimitAngle_sub18F214(pPlayerPed) == 1) {
            *(*gForceCamBehindPlayer_ptr) = 1;
        } else {
            // Check whether force-behind was already set and whether ped is moving
            if (*(*gForceCamBehindPlayer_ptr)) {
                float spdX = *(float*)((char*)pTargetPed + 0x48);
                float spdY = *(float*)((char*)pTargetPed + 0x4C);
                float spdSq= spdX*spdX + spdY*spdY;

                if (spdSq > 0.001f) {
                    // Camera far enough from "behind" angle
                    float fAbsDiff = fabsf(fAlphaDiff);
                    if (fAbsDiff > 0.01f) {
                        // Check facing direction
                        float fFacing = GetAngleBetweenVectors_sub1983CC(pPlayerPed, pTargetPed, 0);
                        if (fFacing == 0.0f) {
                            bool bOnSurface = sub_18E310(pPlayerPed, pTargetPed, 0);
                            if (!bOnSurface)
                                *(*gForceCamBehindPlayer_ptr) = 0;
                        }
                    }
                }
            }
        }
    }

// ── 19. Angular velocity for heading (fAngVel) ────────────────────────
    // (loc_3C35E4 – loc_3C36F4 area)
    {
        float fAlphaDiff = *(float*)((char*)this + 0x84);      // re-read (updated above)
        // ... lots of smoothing using PEDCAM_SET[0x28] / [0x2C] for accel rates
        float fTimeStep  = *CTimer::ms_fTimeStep;
        float fAccelRate = pCS[0x2C / 4];                      // S6 [pCS+0x2C]
        float fDecelRate = pCS[0x28 / 4];                      // S4 [pCS+0x28]
        float fAngVelRaw = fTimeStep * fAccelRate;             // S26 = fTimeStep * fAccel
        float fAngVelSlow= fDecelRate * fTimeStep;             // S2

        // If force-behind OR ped is not in vehicle: scale by S28
        if (S28 != 0.0f || /* bResetCam */ bResetCam) {
            fAngVelRaw *= 2.0f;        // VADD S26, S26, S26
            fAngVelSlow *= 0.5f;       // VMUL S2, S2, #0.5
        }

        // Velocity based on target ped's speed (if nearby vehicle)
        float fVehSpeedContrib = 0.0f;
        {
            CVehicle* pNearVeh = (CVehicle*)*(uintptr_t*)((char*)pTargetPed + 0x56C);
            float fSpeedBase;
            if (pNearVeh) {
                float dvx = *(float*)((char*)pTargetPed + 0x48) - *(float*)((char*)pNearVeh + 0x48);
                float dvy = *(float*)((char*)pTargetPed + 0x4C) - *(float*)((char*)pNearVeh + 0x4C);
                float dvz = *(float*)((char*)pTargetPed + 0x50) - *(float*)((char*)pNearVeh + 0x50);
                fVehSpeedContrib = dvx*dvx + dvy*dvy + dvz*dvz; // will be sqrt'd later
                fSpeedBase = fVehSpeedContrib; // VADD S4,S4,S6; VADD S4,S4,S7 (components)
            } else {
                float vx2 = *(float*)((char*)pTargetPed + 0x48);
                float vy2 = *(float*)((char*)pTargetPed + 0x4C);
                float vz2 = *(float*)((char*)pTargetPed + 0x50);
                fSpeedBase = vx2*vx2 + vy2*vy2 + vz2*vz2;
            }
            fVehSpeedContrib = sqrtf(fSpeedBase);
            fAngVelSlow *= fVehSpeedContrib;               // VMUL S2, S2, S4
        }

        // Clamp and apply delta
        float fAngDelta = fAlphaDiff;                      // S0 from VSUB S0, S18, S20
        float fAngVelFinal = fAngVelSlow;
        fAngVelFinal = (fAngVelFinal >  fAngVelRaw) ?  fAngVelRaw : fAngVelFinal;
        fAngVelFinal = (fAngVelFinal < -fAngVelRaw) ? -fAngVelRaw : fAngVelFinal;
    }

// ── 20. Heading bounds / wrap and heading-angle update ─────────────────
    // (loc_3C36F4 – loc_3C3854)
    {
        float fStoredAlpha  = *(float*)((char*)this + 0x94);   // S30
        float fDeltaAlpha;  // S19 = S20 + S0   (target + angular_velocity)
        // wrap fDeltaAlpha into (camAlpha - π, camAlpha + π]
        float fUpper = fStoredAlpha + 3.1416f;
        float fLower = fStoredAlpha - 3.1416f;
        // if fDeltaAlpha > fUpper: -= 6.2832
        // if fDeltaAlpha < fLower: += 6.2832

        // Compute cos-of-heading for camera source smoothing
        // (smoothes towards target using CTimer::ms_fTimeStep)
        float fTimeStepH = *CTimer::ms_fTimeStep;
        float fCamFront  = *(float*)((char*)this + 0x170);     // S0 (z-component)

        // Clamp front Z to [-1, +1]
        float fFrontClamped = fCamFront;
        if (fFrontClamped >  1.0f) fFrontClamped =  1.0f;
        if (fFrontClamped < -1.0f) fFrontClamped = -1.0f;

        // sub_19AA70 = acosf-like (acos in float-int calling conv)
        float fPitchAngle = acosf_sub19AA70(fFrontClamped);    // in S18 after call
        // (used to transition camera when ped turns quickly)
    }

// ── 21. Final source position computation (collision pass 1) ──────────
    // (loc_3C3732 onwards → matrix build, NormaliseVector, etc.)
    {
        float fAlphaFinal = *(float*)((char*)this + 0x94);
        float fBetaFinal  = *(float*)((char*)this + 0x84);

        float fSinA = sinf_sub18C778(fAlphaFinal);
        float fCosA = cosf_sub18C778(fAlphaFinal);  // hmm, sin/cos alternate
        float fCosB = cosf_sub18F664(fAlphaFinal);  // per assembly reuse
        float fSinB = sinf_sub18C778(fBetaFinal);

        // Camera Source (position)
        float fDistFinal = *(float*)((char*)this + 0x17C);     // dist along view axis
        float fSrc0X = fTgtX + fDistFinal * fSinA;
        float fSrc0Y = fTgtY + fDistFinal * fCosA;
        float fSrc0Z = fCamPosZ - fDistFinal * fCosB;

        *(float*)((char*)this + 0x1B0) = fSrc0X;
        *(float*)((char*)this + 0x1B4) = fSrc0Y;
        *(float*)((char*)this + 0x1B8) = fSrc0Z;
        *(float*)((char*)this + 0x1BC) = /* near target */ fTgtX;
        *(float*)((char*)this + 0x1C0) = fTgtY;
        *(float*)((char*)this + 0x1C4) = fCamPosZ;
    }

// ── 22. Stick / touch input → update heading & pitch ──────────────────
    // (loc_3C3BD2 onwards)
    {
        float fAimSens = *(*AIMWEAPON_STICK_SENS_ptr);
        float fFOVScale = *(float*)((char*)this + 0x8C) / 80.0f;   // VDIV S0, [this+0x8C], 80

        // Horizontal stick
        float fStickHInput; // from sub_18E310 or touch handlers (SP+0x68)
        // Vertical stick
        float fStickVInput; // SP+0x64

        // Scale by sensitivity² and FOV
        float fSensSq = fAimSens * fAimSens;
        fStickHInput *= fSensSq * fabsf(fStickHInput) * (fFOVScale * 0.071429f);
        fStickVInput *= fSensSq * fabsf(fStickVInput) * (fFOVScale * 0.042857f);

        // Read input type from GetInputMode
        int nInputMode = sub_18A940();

        if (nInputMode == 2) {
            // Gamepad / D-pad mode
            // ... sub_1983CC / sub_18E310 for raw gamepad axes ...
        } else if (nInputMode == 1) {
            // Touchscreen mode
            // ... update from MobileSettings and NewMouseControllerState ...
        } else {
            // Mouse mode
            float fMouseSens = *(float*)((char*)MobileSettings + 0x3E8);
            float fMouseY    = *(float*)((char*)NewMouseControllerState + 0xC);
            fStickVInput = fMouseSens * fMouseY * (-80.0f / *(float*)((char*)this + 0x8C));
        }

        // Update heading (this+0x94)
        float fAlphaN = *(float*)((char*)this + 0x94);
        float fAlphaNew = fAlphaN + fStickHInput;
        // Wrap to (−π, π]
        if (fAlphaNew >  3.1416f) fAlphaNew -= 6.2832f;
        if (fAlphaNew < -3.1416f) fAlphaNew += 6.2832f;
        *(float*)((char*)this + 0x94) = fAlphaNew;

        // Update pitch (this+0x88)
        float fPitchN   = *(float*)((char*)this + 0x88);
        float fPitchNew = fPitchN * fAimSens*fAimSens + fStickVInput * fFOVScale;
        fPitchNew = (fPitchNew >  fMinHeightAng) ?  fMinHeightAng : fPitchNew;
        fPitchNew = (fPitchNew < -fMinHeightAng) ? -fMinHeightAng : fPitchNew;
        if (fabsf(fPitchNew) < 0.0001f) fPitchNew = 0.0f;
        *(float*)((char*)this + 0x88) = fPitchNew;
    }

// ── 23. Collision detection + final position ──────────────────────────
    // (loc_3C3D90 – loc_3C4C5C)
    {
        // Smooth target alpha using PEDCAM_SET[0x24] (fSmoothAlphaRate)
        float fSmAlpha = sub_19C928(/* lerp alpha */);          // [R4+dword_6AA034 offset]
        float fTargetAlpha = pCS[0x24 / 4];

        // Build final cam orientation matrix via sin/cos of heading+pitch
        float fAlphaF = *(float*)((char*)this + 0x94);
        float fBetaF  = *(float*)((char*)this + 0x84);

        float fSinAF = sinf_sub18C778(fAlphaF);
        float fCosAF = cosf_sub18C778(fAlphaF);   // note: using sin for cos register via sequence
        float fCosBF = cosf_sub18F664(fBetaF);

        // Re-compute final source position
        float fDistF = *(float*)((char*)this + 0x17C);
        *(float*)((char*)this + 0x1B0) = fTgtX + fDistF * fSinAF * fCosBF;
        *(float*)((char*)this + 0x1B4) = fTgtY - fDistF * fCosAF * fCosBF;
        *(float*)((char*)this + 0x1B8) = fCamPosZ - fDistF * sinf_sub18C778(fBetaF);

        // Normalise direction vectors stored at 0x168, 0x16C, 0x170
        CVector vecDir = {
            fTgtX - *(float*)((char*)this + 0x1B0),
            fTgtY - *(float*)((char*)this + 0x1B4),
            fCamPosZ - *(float*)((char*)this + 0x1B8)
        };
        NormaliseVector3D_sub19BA04(&vecDir);
        *(float*)((char*)this + 0x168) = vecDir.x;
        *(float*)((char*)this + 0x16C) = vecDir.y;
        *(float*)((char*)this + 0x170) = vecDir.z;

        // Camera mode == 3 → full collision sweep via CCollision
        if (*(int*)dword_6A9F18 == 3) {
            CCollision::bCamCollideWithObjects = true;
            CCollision::bCamCollideWithVehicles = true;
            CCollision::bCamCollideWithPeds = true;

            // CCam::ProcessOccluder-style collision
            sub_18AE74(/* source, target, radius, result */);

            // Apply collision-corrected distances
            // (stored back at this+0x174..0x17C and this+0x1B0..0x1C4)
        }

        // Normalise final front direction
        // build up-vector via cross product (sub_18B818)
        CVector3x3 mFinal;
        sub_18B818(&mFinal, /* forward */ nullptr, /* up-hint */ nullptr);
        // Store result as full 3×3 at this+0x18C..0x194

        // STRH R5(=0), [R0, word_951FC2 offset]  → clear some CCamera flags
        *(uint16_t*)((char*)TheCamera + (0x951FC2 - 0x951FA8)) = 0;
    }

// ── 24. Idle cam: check if we should trigger it ───────────────────────
    // (loc_3C4B7C – loc_3C4C5C)
    {
        if (!bResetCam && *CGame::currArea == 0) {
            // Check if ped is stationary
            float vx = *(float*)((char*)pTargetPed + 0x48);
            float vy = *(float*)((char*)pTargetPed + 0x4C);
            float vz = *(float*)((char*)pTargetPed + 0x50);
            float fSpeedSq = vx*vx + vy*vy + vz*vz;

            if (fSpeedSq <= 0.0001f) {
                // Check current pad for input
                CPad* pPad = *(*currentPad_ptr);
                uint32_t nPadInput = *(uint32_t*)((char*)pPad);

                if (nPadInput == 0)
                    FindPlayerPed_sub19F8F4(0);  // update idle state

                // gIdleCam: compare stored ped ID with current, update timer
                uint32_t nStoredPedID = *(uint32_t*)((char*)*gIdleCam_ptr + (0x952D4C - 0x952CBC));
                uint32_t nCurPedID    = *(uint32_t*)((char*)*currentPad + 0x134);

                if (nStoredPedID == nCurPedID) {
                    // Increment idle counter
                    float fIdleTick = *CTimer::ms_fTimeStep / 50.0f * 1000.0f;
                    *(uint32_t*)((char*)*gIdleCam_ptr + (0x952D50 - 0x952CBC)) += (uint32_t)fIdleTick;
                } else {
                    // Reset idle counter to 0, store current ped
                    *(uint32_t*)((char*)*gIdleCam_ptr + (0x952D4C - 0x952CBC)) = nCurPedID;
                }
            }
        }

        // Always update gIdleCam timer store
        *(uint32_t*)((char*)*gIdleCam_ptr + (0x952D50 - 0x952CBC)) = /* computed value */ 0;
        *(uint8_t*)((char*)this + 0xA) = 0;  // STRB.W R4, [R10,#0xA]
    }

// ── 25. Epilogue ───────────────────────────────────────────────────────
loc_3C4C60:
    // SP += 0xA0
    // VPOP {D8-D15}
    // SP += 4
    // POP.W {R8-R11}
    // POP {R4-R7, PC}
    return;

// Unreachable after goto – but present in assembly as dead-code target
loc_3C30A8:
    // Called when bResetCam path was taken but no active vehicle found
    {
        // Get nearest ped/entity for heading reset
        // sub_198480(0xFFFFFFFF)  → find closest ped
        CPed* pNearest = (CPed*)sub_198480(0xFFFFFFFF);
        if (!pNearest) goto loc_3C30E8;

        // Get active ped from pool
        CPed* pActive = (CPed*)sub_19433C(0xFFFFFFFF, 0);
        if (!pActive) goto loc_3C30E8;

        // If ped type falls in range [3,5] → use bResetCam path
        int nPT = *(int*)((char*)pActive + 0x5A4) - 3;
        if ((unsigned)nPT < 3) goto loc_3C2F8A;  // BCC

        // Check currentPad for input
        CPad* pCP = *(*currentPad_ptr);
        uint32_t nInput = *(uint32_t*)*pCP;
        if (nInput == 0) FindPlayerPed(0);

        // GetControllerButtonState
        if (sub_1913A4() == 0) goto loc_3C2F8A;

loc_3C30E8:
        // Check CCamera flags for reverse-view
        uint8_t bRevA = *(uint8_t*)((char*)TheCamera + (0x951FC2 - 0x951FA8));
        uint8_t bRevB = *(uint8_t*)((char*)TheCamera + (0x951FC2 + 1 - 0x951FA8));
        if ((bRevA | bRevB) << 24) goto loc_3C2F8A;

        // bResetCam forced or ped on horse?
        if (bResetCam == 1) goto loc_3C2F8A;

        // Check ped's nearby vehicle at 0x56C
        CVehicle* pSideVeh = (CVehicle*)*(uintptr_t*)((char*)pTargetPed + 0x56C);
        if (!pSideVeh) goto loc_3C3448;

        // Validate ped-type and model
        if ((*(uint8_t*)((char*)pSideVeh + 0x3A) & 7) == 2) {
            uint32_t nVehPT = *(uint32_t*)((char*)pSideVeh + 0x5A4);
            if (nVehPT == 6) goto loc_3C3140;
        }

        {
            void* pLink = *(void**)((char*)pSideVeh + 0x100);
            if (!pLink) goto loc_3C3448;
            if ((*(uint8_t*)((char*)pLink + 0x3A) & 7) == 2) {
                if (*(uint32_t*)((char*)pLink + 0x5A4) == 6) goto loc_3C3140;
            }
            goto loc_3C3448;
        }

loc_3C3140:
        // Ped near a bike/horse vehicle — update "on-vehicle" position velocity
        {
            float vx2 = *(float*)((char*)pSideVeh + 0x48);
            float vy2 = *(float*)((char*)pSideVeh + 0x4C);
            float vz2 = *(float*)((char*)pSideVeh + 0x50);
            float fSpdSq = vx2*vx2 + vy2*vy2 + vz2*vz2;
            float fSpd   = sqrtf(fSpdSq);
            float fClamp = (fSpd - 0.01f) / fSpd;   // VSQRT, then (spd - 0.01) / spd
            fClamp = (fClamp > 0.0f) ? fClamp : 0.0f;

            float fTimeStepLS = *CTimer::ms_fTimeStep;

            // Update ped's stored velocity  (this+0x1B0..0x1B8 range)
            float* pVelStore = (float*)((char*)this + 0x1B0);
            float* pPedOldV  = (float*)((char*)this + 0x1BC);
            pVelStore[0] += fClamp * vx2 * fTimeStepLS;
            pVelStore[1] += fClamp * vy2 * fTimeStepLS;
            pVelStore[2] += fClamp * vz2 * fTimeStepLS;

            // Similar for nearby vehicle velocity contribution
        }
    }

loc_3C3448:
    // Normalize direction vector and re-enter main flow
    goto loc_3C3336;
}

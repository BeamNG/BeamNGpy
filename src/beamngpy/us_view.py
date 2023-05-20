import math
import numpy as np

class US_View:

    def __init__(self, bng, vehicle, us0, us1, us2, us3, us4, us5, us6, us7, us8, us9, us10, us11):

        # Set up the BeamNG state.
        self.bng = bng
        self.vehicle = vehicle

        # Set up the Ultrasonic sensors.
        self.us_FL, self.us_FR, self.us_BL, self.us_BR, self.us_ML, self.us_MR = us0, us1, us2, us3, us4, us5
        self.us_FL2, self.us_FR2, self.us_BL2, self.us_BR2, self.us_ML2, self.us_MR2 = us6, us7, us8, us9, us10, us11

        # Set up the geometric coordinates of the primitives.
        gap = 10.0
        ML_x_offset = 785.0                                                                                                         # Top side-bar. LEFT
        ML_y_offset = 700.0
        ML_w = 155.0
        ML_h = 15.0
        ML_h_gap = ML_h + gap
        loc_ML_0 = [ML_x_offset, ML_y_offset, ML_x_offset + ML_w, ML_y_offset + ML_h]
        loc_ML_1 = [ML_x_offset, ML_y_offset + ML_h_gap, ML_x_offset + ML_w, ML_y_offset + ML_h_gap + ML_h]
        loc_ML_2 = [ML_x_offset, ML_y_offset + (2.0 * ML_h_gap), ML_x_offset + ML_w, ML_y_offset + (2.0 * ML_h_gap) + ML_h]
        loc_ML_3 = [ML_x_offset, ML_y_offset + (3.0 * ML_h_gap), ML_x_offset + ML_w, ML_y_offset + (3.0 * ML_h_gap) + ML_h]
        loc_ML_4 = [ML_x_offset, ML_y_offset + (4.0 * ML_h_gap), ML_x_offset + ML_w, ML_y_offset + (4.0 * ML_h_gap) + ML_h]
        loc_ML_5 = [ML_x_offset, ML_y_offset + (5.0 * ML_h_gap), ML_x_offset + ML_w, ML_y_offset + (5.0 * ML_h_gap) + ML_h]
        loc_ML_6 = [ML_x_offset, ML_y_offset + (6.0 * ML_h_gap), ML_x_offset + ML_w, ML_y_offset + (6.0 * ML_h_gap) + ML_h]
        MR_x_offset = 785.0                                                                                                         # Bottom side-bar. LEFT
        MR_y_offset = 330.0
        MR_w = 155.0
        MR_h = 15.0
        MR_h_gap = MR_h + gap
        loc_MR_0 = [MR_x_offset, MR_y_offset, MR_x_offset + MR_w, MR_y_offset - MR_h]
        loc_MR_1 = [MR_x_offset, MR_y_offset - MR_h_gap, MR_x_offset + MR_w, MR_y_offset - MR_h_gap - MR_h]
        loc_MR_2 = [MR_x_offset, MR_y_offset - (2.0 * MR_h_gap), MR_x_offset + MR_w, MR_y_offset - (2.0 * MR_h_gap) - MR_h]
        loc_MR_3 = [MR_x_offset, MR_y_offset - (3.0 * MR_h_gap), MR_x_offset + MR_w, MR_y_offset - (3.0 * MR_h_gap) - MR_h]
        loc_MR_4 = [MR_x_offset, MR_y_offset - (4.0 * MR_h_gap), MR_x_offset + MR_w, MR_y_offset - (4.0 * MR_h_gap) - MR_h]
        loc_MR_5 = [MR_x_offset, MR_y_offset - (5.0 * MR_h_gap), MR_x_offset + MR_w, MR_y_offset - (5.0 * MR_h_gap) - MR_h]
        loc_MR_6 = [MR_x_offset, MR_y_offset - (6.0 * MR_h_gap), MR_x_offset + MR_w, MR_y_offset - (6.0 * MR_h_gap) - MR_h]

        ML2_x_offset = 960.0                                                                                                         # Top side-bar. RIGHT
        ML2_y_offset = 700.0
        ML2_w = 155.0
        ML2_h = 15.0
        ML2_h_gap = ML2_h + gap
        loc_ML2_0 = [ML2_x_offset, ML2_y_offset, ML2_x_offset + ML2_w, ML2_y_offset + ML2_h]
        loc_ML2_1 = [ML2_x_offset, ML2_y_offset + ML2_h_gap, ML2_x_offset + ML2_w, ML2_y_offset + ML2_h_gap + ML2_h]
        loc_ML2_2 = [ML2_x_offset, ML2_y_offset + (2.0 * ML2_h_gap), ML2_x_offset + ML2_w, ML2_y_offset + (2.0 * ML2_h_gap) + ML2_h]
        loc_ML2_3 = [ML2_x_offset, ML2_y_offset + (3.0 * ML2_h_gap), ML2_x_offset + ML2_w, ML2_y_offset + (3.0 * ML2_h_gap) + ML2_h]
        loc_ML2_4 = [ML2_x_offset, ML2_y_offset + (4.0 * ML2_h_gap), ML2_x_offset + ML2_w, ML2_y_offset + (4.0 * ML2_h_gap) + ML2_h]
        loc_ML2_5 = [ML2_x_offset, ML2_y_offset + (5.0 * ML2_h_gap), ML2_x_offset + ML2_w, ML2_y_offset + (5.0 * ML2_h_gap) + ML2_h]
        loc_ML2_6 = [ML2_x_offset, ML2_y_offset + (6.0 * ML2_h_gap), ML2_x_offset + ML2_w, ML2_y_offset + (6.0 * ML2_h_gap) + ML2_h]
        MR2_x_offset = 960.0                                                                                                         # Bottom side-bar. RIGHT
        MR2_y_offset = 330.0
        MR2_w = 155.0
        MR2_h = 15.0
        MR2_h_gap = MR2_h + gap
        loc_MR2_0 = [MR2_x_offset, MR2_y_offset, MR2_x_offset + MR2_w, MR2_y_offset - MR2_h]
        loc_MR2_1 = [MR2_x_offset, MR2_y_offset - MR2_h_gap, MR2_x_offset + MR2_w, MR2_y_offset - MR2_h_gap - MR2_h]
        loc_MR2_2 = [MR2_x_offset, MR2_y_offset - (2.0 * MR2_h_gap), MR2_x_offset + MR2_w, MR2_y_offset - (2.0 * MR2_h_gap) - MR2_h]
        loc_MR2_3 = [MR2_x_offset, MR2_y_offset - (3.0 * MR2_h_gap), MR2_x_offset + MR2_w, MR2_y_offset - (3.0 * MR2_h_gap) - MR2_h]
        loc_MR2_4 = [MR2_x_offset, MR2_y_offset - (4.0 * MR2_h_gap), MR2_x_offset + MR2_w, MR2_y_offset - (4.0 * MR2_h_gap) - MR2_h]
        loc_MR2_5 = [MR2_x_offset, MR2_y_offset - (5.0 * MR2_h_gap), MR2_x_offset + MR2_w, MR2_y_offset - (5.0 * MR2_h_gap) - MR2_h]
        loc_MR2_6 = [MR2_x_offset, MR2_y_offset - (6.0 * MR2_h_gap), MR2_x_offset + MR2_w, MR2_y_offset - (6.0 * MR2_h_gap) - MR2_h]

        # Top-right arc.
        div = 50
        q_pi = math.pi * 0.25
        div_f = (q_pi - 0.08) / float(div)
        wid = 15
        cx, cy = 1115, 525
        wid_gap = wid + gap
        r0, r1, r2, r3, r4, r5, r6 = 190, 190 + wid_gap, 190 + (2 * wid_gap), 190 + (3 * wid_gap), 190 + (4 * wid_gap), 190 + (5 * wid_gap), 190 + (6 * wid_gap)
        rr0, rr1, rr2, rr3, rr4, rr5, rr6 = r0 - wid, r1 - wid, r2 - wid, r3 - wid, r4 - wid, r5 - wid, r6 - wid
        TR_tx0, TR_tx1, TR_tx2, TR_tx3, TR_tx4, TR_tx5, TR_tx6 = [], [], [], [], [], [], []
        TR_ty0, TR_ty1, TR_ty2, TR_ty3, TR_ty4, TR_ty5, TR_ty6 = [], [], [], [], [], [], []
        TR2_tx0, TR2_tx1, TR2_tx2, TR2_tx3, TR2_tx4, TR2_tx5, TR2_tx6 = [], [], [], [], [], [], []
        TR2_ty0, TR2_ty1, TR2_ty2, TR2_ty3, TR2_ty4, TR2_ty5, TR2_ty6 = [], [], [], [], [], [], []
        for i in range(div + 1):
            ang = i * div_f
            ca, sa = math.cos(ang), math.sin(ang)
            TR_tx0.append((r0 * ca) + cx)
            TR_tx1.append((r1 * ca) + cx)
            TR_tx2.append((r2 * ca) + cx)
            TR_tx3.append((r3 * ca) + cx)
            TR_tx4.append((r4 * ca) + cx)
            TR_tx5.append((r5 * ca) + cx)
            TR_tx6.append((r6 * ca) + cx)
            TR_ty0.append((r0 * sa) + cy)
            TR_ty1.append((r1 * sa) + cy)
            TR_ty2.append((r2 * sa) + cy)
            TR_ty3.append((r3 * sa) + cy)
            TR_ty4.append((r4 * sa) + cy)
            TR_ty5.append((r5 * sa) + cy)
            TR_ty6.append((r6 * sa) + cy)

            TR2_tx0.append((rr0 * ca) + cx)
            TR2_tx1.append((rr1 * ca) + cx)
            TR2_tx2.append((rr2 * ca) + cx)
            TR2_tx3.append((rr3 * ca) + cx)
            TR2_tx4.append((rr4 * ca) + cx)
            TR2_tx5.append((rr5 * ca) + cx)
            TR2_tx6.append((rr6 * ca) + cx)
            TR2_ty0.append((rr0 * sa) + cy)
            TR2_ty1.append((rr1 * sa) + cy)
            TR2_ty2.append((rr2 * sa) + cy)
            TR2_ty3.append((rr3 * sa) + cy)
            TR2_ty4.append((rr4 * sa) + cy)
            TR2_ty5.append((rr5 * sa) + cy)
            TR2_ty6.append((rr6 * sa) + cy)

        T2R_tx0, T2R_tx1, T2R_tx2, T2R_tx3, T2R_tx4, T2R_tx5, T2R_tx6 = [], [], [], [], [], [], []
        T2R_ty0, T2R_ty1, T2R_ty2, T2R_ty3, T2R_ty4, T2R_ty5, T2R_ty6 = [], [], [], [], [], [], []
        T2R2_tx0, T2R2_tx1, T2R2_tx2, T2R2_tx3, T2R2_tx4, T2R2_tx5, T2R2_tx6 = [], [], [], [], [], [], []
        T2R2_ty0, T2R2_ty1, T2R2_ty2, T2R2_ty3, T2R2_ty4, T2R2_ty5, T2R2_ty6 = [], [], [], [], [], [], []
        for i in range(div + 1):
            ang = i * div_f + q_pi
            ca, sa = math.cos(ang), math.sin(ang)
            T2R_tx0.append((r0 * ca) + cx)
            T2R_tx1.append((r1 * ca) + cx)
            T2R_tx2.append((r2 * ca) + cx)
            T2R_tx3.append((r3 * ca) + cx)
            T2R_tx4.append((r4 * ca) + cx)
            T2R_tx5.append((r5 * ca) + cx)
            T2R_tx6.append((r6 * ca) + cx)
            T2R_ty0.append((r0 * sa) + cy)
            T2R_ty1.append((r1 * sa) + cy)
            T2R_ty2.append((r2 * sa) + cy)
            T2R_ty3.append((r3 * sa) + cy)
            T2R_ty4.append((r4 * sa) + cy)
            T2R_ty5.append((r5 * sa) + cy)
            T2R_ty6.append((r6 * sa) + cy)

            T2R2_tx0.append((rr0 * ca) + cx)
            T2R2_tx1.append((rr1 * ca) + cx)
            T2R2_tx2.append((rr2 * ca) + cx)
            T2R2_tx3.append((rr3 * ca) + cx)
            T2R2_tx4.append((rr4 * ca) + cx)
            T2R2_tx5.append((rr5 * ca) + cx)
            T2R2_tx6.append((rr6 * ca) + cx)
            T2R2_ty0.append((rr0 * sa) + cy)
            T2R2_ty1.append((rr1 * sa) + cy)
            T2R2_ty2.append((rr2 * sa) + cy)
            T2R2_ty3.append((rr3 * sa) + cy)
            T2R2_ty4.append((rr4 * sa) + cy)
            T2R2_ty5.append((rr5 * sa) + cy)
            T2R2_ty6.append((rr6 * sa) + cy)

        # Bottom-right arc.
        cx, cy = 1115, 505
        BR_tx0, BR_tx1, BR_tx2, BR_tx3, BR_tx4, BR_tx5, BR_tx6 = [], [], [], [], [], [], []
        BR_ty0, BR_ty1, BR_ty2, BR_ty3, BR_ty4, BR_ty5, BR_ty6 = [], [], [], [], [], [], []
        BR2_tx0, BR2_tx1, BR2_tx2, BR2_tx3, BR2_tx4, BR2_tx5, BR2_tx6 = [], [], [], [], [], [], []
        BR2_ty0, BR2_ty1, BR2_ty2, BR2_ty3, BR2_ty4, BR2_ty5, BR2_ty6 = [], [], [], [], [], [], []
        for i in range(div + 1):
            ang = -i * div_f
            ca, sa = math.cos(ang), math.sin(ang)
            BR_tx0.append((r0 * ca) + cx)
            BR_tx1.append((r1 * ca) + cx)
            BR_tx2.append((r2 * ca) + cx)
            BR_tx3.append((r3 * ca) + cx)
            BR_tx4.append((r4 * ca) + cx)
            BR_tx5.append((r5 * ca) + cx)
            BR_tx6.append((r6 * ca) + cx)
            BR_ty0.append((r0 * sa) + cy)
            BR_ty1.append((r1 * sa) + cy)
            BR_ty2.append((r2 * sa) + cy)
            BR_ty3.append((r3 * sa) + cy)
            BR_ty4.append((r4 * sa) + cy)
            BR_ty5.append((r5 * sa) + cy)
            BR_ty6.append((r6 * sa) + cy)

            BR2_tx0.append((rr0 * ca) + cx)
            BR2_tx1.append((rr1 * ca) + cx)
            BR2_tx2.append((rr2 * ca) + cx)
            BR2_tx3.append((rr3 * ca) + cx)
            BR2_tx4.append((rr4 * ca) + cx)
            BR2_tx5.append((rr5 * ca) + cx)
            BR2_tx6.append((rr6 * ca) + cx)
            BR2_ty0.append((rr0 * sa) + cy)
            BR2_ty1.append((rr1 * sa) + cy)
            BR2_ty2.append((rr2 * sa) + cy)
            BR2_ty3.append((rr3 * sa) + cy)
            BR2_ty4.append((rr4 * sa) + cy)
            BR2_ty5.append((rr5 * sa) + cy)
            BR2_ty6.append((rr6 * sa) + cy)

        B2R_tx0, B2R_tx1, B2R_tx2, B2R_tx3, B2R_tx4, B2R_tx5, B2R_tx6 = [], [], [], [], [], [], []
        B2R_ty0, B2R_ty1, B2R_ty2, B2R_ty3, B2R_ty4, B2R_ty5, B2R_ty6 = [], [], [], [], [], [], []
        B2R2_tx0, B2R2_tx1, B2R2_tx2, B2R2_tx3, B2R2_tx4, B2R2_tx5, B2R2_tx6 = [], [], [], [], [], [], []
        B2R2_ty0, B2R2_ty1, B2R2_ty2, B2R2_ty3, B2R2_ty4, B2R2_ty5, B2R2_ty6 = [], [], [], [], [], [], []
        for i in range(div + 1):
            ang = -i * div_f - q_pi
            ca, sa = math.cos(ang), math.sin(ang)
            B2R_tx0.append((r0 * ca) + cx)
            B2R_tx1.append((r1 * ca) + cx)
            B2R_tx2.append((r2 * ca) + cx)
            B2R_tx3.append((r3 * ca) + cx)
            B2R_tx4.append((r4 * ca) + cx)
            B2R_tx5.append((r5 * ca) + cx)
            B2R_tx6.append((r6 * ca) + cx)
            B2R_ty0.append((r0 * sa) + cy)
            B2R_ty1.append((r1 * sa) + cy)
            B2R_ty2.append((r2 * sa) + cy)
            B2R_ty3.append((r3 * sa) + cy)
            B2R_ty4.append((r4 * sa) + cy)
            B2R_ty5.append((r5 * sa) + cy)
            B2R_ty6.append((r6 * sa) + cy)

            B2R2_tx0.append((rr0 * ca) + cx)
            B2R2_tx1.append((rr1 * ca) + cx)
            B2R2_tx2.append((rr2 * ca) + cx)
            B2R2_tx3.append((rr3 * ca) + cx)
            B2R2_tx4.append((rr4 * ca) + cx)
            B2R2_tx5.append((rr5 * ca) + cx)
            B2R2_tx6.append((rr6 * ca) + cx)
            B2R2_ty0.append((rr0 * sa) + cy)
            B2R2_ty1.append((rr1 * sa) + cy)
            B2R2_ty2.append((rr2 * sa) + cy)
            B2R2_ty3.append((rr3 * sa) + cy)
            B2R2_ty4.append((rr4 * sa) + cy)
            B2R2_ty5.append((rr5 * sa) + cy)
            B2R2_ty6.append((rr6 * sa) + cy)

        # Top-left arc.
        cx, cy = 785, 525
        TL_tx0, TL_tx1, TL_tx2, TL_tx3, TL_tx4, TL_tx5, TL_tx6 = [], [], [], [], [], [], []
        TL_ty0, TL_ty1, TL_ty2, TL_ty3, TL_ty4, TL_ty5, TL_ty6 = [], [], [], [], [], [], []
        TL2_tx0, TL2_tx1, TL2_tx2, TL2_tx3, TL2_tx4, TL2_tx5, TL2_tx6 = [], [], [], [], [], [], []
        TL2_ty0, TL2_ty1, TL2_ty2, TL2_ty3, TL2_ty4, TL2_ty5, TL2_ty6 = [], [], [], [], [], [], []
        for i in range(div + 1):
            ang = i * div_f
            ca, sa = math.cos(ang), math.sin(ang)
            TL_tx0.append(cx - (r0 * ca))
            TL_tx1.append(cx - (r1 * ca))
            TL_tx2.append(cx - (r2 * ca))
            TL_tx3.append(cx - (r3 * ca))
            TL_tx4.append(cx - (r4 * ca))
            TL_tx5.append(cx - (r5 * ca))
            TL_tx6.append(cx - (r6 * ca))
            TL_ty0.append((r0 * sa) + cy)
            TL_ty1.append((r1 * sa) + cy)
            TL_ty2.append((r2 * sa) + cy)
            TL_ty3.append((r3 * sa) + cy)
            TL_ty4.append((r4 * sa) + cy)
            TL_ty5.append((r5 * sa) + cy)
            TL_ty6.append((r6 * sa) + cy)

            TL2_tx0.append(cx - (rr0 * ca))
            TL2_tx1.append(cx - (rr1 * ca))
            TL2_tx2.append(cx - (rr2 * ca))
            TL2_tx3.append(cx - (rr3 * ca))
            TL2_tx4.append(cx - (rr4 * ca))
            TL2_tx5.append(cx - (rr5 * ca))
            TL2_tx6.append(cx - (rr6 * ca))
            TL2_ty0.append((rr0 * sa) + cy)
            TL2_ty1.append((rr1 * sa) + cy)
            TL2_ty2.append((rr2 * sa) + cy)
            TL2_ty3.append((rr3 * sa) + cy)
            TL2_ty4.append((rr4 * sa) + cy)
            TL2_ty5.append((rr5 * sa) + cy)
            TL2_ty6.append((rr6 * sa) + cy)

        T2L_tx0, T2L_tx1, T2L_tx2, T2L_tx3, T2L_tx4, T2L_tx5, T2L_tx6 = [], [], [], [], [], [], []
        T2L_ty0, T2L_ty1, T2L_ty2, T2L_ty3, T2L_ty4, T2L_ty5, T2L_ty6 = [], [], [], [], [], [], []
        T2L2_tx0, T2L2_tx1, T2L2_tx2, T2L2_tx3, T2L2_tx4, T2L2_tx5, T2L2_tx6 = [], [], [], [], [], [], []
        T2L2_ty0, T2L2_ty1, T2L2_ty2, T2L2_ty3, T2L2_ty4, T2L2_ty5, T2L2_ty6 = [], [], [], [], [], [], []
        for i in range(div + 1):
            ang = i * div_f + q_pi
            ca, sa = math.cos(ang), math.sin(ang)
            T2L_tx0.append(cx - (r0 * ca))
            T2L_tx1.append(cx - (r1 * ca))
            T2L_tx2.append(cx - (r2 * ca))
            T2L_tx3.append(cx - (r3 * ca))
            T2L_tx4.append(cx - (r4 * ca))
            T2L_tx5.append(cx - (r5 * ca))
            T2L_tx6.append(cx - (r6 * ca))
            T2L_ty0.append((r0 * sa) + cy)
            T2L_ty1.append((r1 * sa) + cy)
            T2L_ty2.append((r2 * sa) + cy)
            T2L_ty3.append((r3 * sa) + cy)
            T2L_ty4.append((r4 * sa) + cy)
            T2L_ty5.append((r5 * sa) + cy)
            T2L_ty6.append((r6 * sa) + cy)

            T2L2_tx0.append(cx - (rr0 * ca))
            T2L2_tx1.append(cx - (rr1 * ca))
            T2L2_tx2.append(cx - (rr2 * ca))
            T2L2_tx3.append(cx - (rr3 * ca))
            T2L2_tx4.append(cx - (rr4 * ca))
            T2L2_tx5.append(cx - (rr5 * ca))
            T2L2_tx6.append(cx - (rr6 * ca))
            T2L2_ty0.append((rr0 * sa) + cy)
            T2L2_ty1.append((rr1 * sa) + cy)
            T2L2_ty2.append((rr2 * sa) + cy)
            T2L2_ty3.append((rr3 * sa) + cy)
            T2L2_ty4.append((rr4 * sa) + cy)
            T2L2_ty5.append((rr5 * sa) + cy)
            T2L2_ty6.append((rr6 * sa) + cy)

        # Bottom-left arc.
        cx, cy = 785, 505
        BL_tx0, BL_tx1, BL_tx2, BL_tx3, BL_tx4, BL_tx5, BL_tx6 = [], [], [], [], [], [], []
        BL_ty0, BL_ty1, BL_ty2, BL_ty3, BL_ty4, BL_ty5, BL_ty6 = [], [], [], [], [], [], []
        BL2_tx0, BL2_tx1, BL2_tx2, BL2_tx3, BL2_tx4, BL2_tx5, BL2_tx6 = [], [], [], [], [], [], []
        BL2_ty0, BL2_ty1, BL2_ty2, BL2_ty3, BL2_ty4, BL2_ty5, BL2_ty6 = [], [], [], [], [], [], []
        for i in range(div + 1):
            ang = -i * div_f
            ca, sa = math.cos(ang), math.sin(ang)
            BL_tx0.append(cx - (r0 * ca))
            BL_tx1.append(cx - (r1 * ca))
            BL_tx2.append(cx - (r2 * ca))
            BL_tx3.append(cx - (r3 * ca))
            BL_tx4.append(cx - (r4 * ca))
            BL_tx5.append(cx - (r5 * ca))
            BL_tx6.append(cx - (r6 * ca))
            BL_ty0.append((r0 * sa) + cy)
            BL_ty1.append((r1 * sa) + cy)
            BL_ty2.append((r2 * sa) + cy)
            BL_ty3.append((r3 * sa) + cy)
            BL_ty4.append((r4 * sa) + cy)
            BL_ty5.append((r5 * sa) + cy)
            BL_ty6.append((r6 * sa) + cy)

            BL2_tx0.append(cx - (rr0 * ca))
            BL2_tx1.append(cx - (rr1 * ca))
            BL2_tx2.append(cx - (rr2 * ca))
            BL2_tx3.append(cx - (rr3 * ca))
            BL2_tx4.append(cx - (rr4 * ca))
            BL2_tx5.append(cx - (rr5 * ca))
            BL2_tx6.append(cx - (rr6 * ca))
            BL2_ty0.append((rr0 * sa) + cy)
            BL2_ty1.append((rr1 * sa) + cy)
            BL2_ty2.append((rr2 * sa) + cy)
            BL2_ty3.append((rr3 * sa) + cy)
            BL2_ty4.append((rr4 * sa) + cy)
            BL2_ty5.append((rr5 * sa) + cy)
            BL2_ty6.append((rr6 * sa) + cy)

        B2L_tx0, B2L_tx1, B2L_tx2, B2L_tx3, B2L_tx4, B2L_tx5, B2L_tx6 = [], [], [], [], [], [], []
        B2L_ty0, B2L_ty1, B2L_ty2, B2L_ty3, B2L_ty4, B2L_ty5, B2L_ty6 = [], [], [], [], [], [], []
        B2L2_tx0, B2L2_tx1, B2L2_tx2, B2L2_tx3, B2L2_tx4, B2L2_tx5, B2L2_tx6 = [], [], [], [], [], [], []
        B2L2_ty0, B2L2_ty1, B2L2_ty2, B2L2_ty3, B2L2_ty4, B2L2_ty5, B2L2_ty6 = [], [], [], [], [], [], []
        for i in range(div + 1):
            ang = -i * div_f - q_pi
            ca, sa = math.cos(ang), math.sin(ang)
            B2L_tx0.append(cx - (r0 * ca))
            B2L_tx1.append(cx - (r1 * ca))
            B2L_tx2.append(cx - (r2 * ca))
            B2L_tx3.append(cx - (r3 * ca))
            B2L_tx4.append(cx - (r4 * ca))
            B2L_tx5.append(cx - (r5 * ca))
            B2L_tx6.append(cx - (r6 * ca))
            B2L_ty0.append((r0 * sa) + cy)
            B2L_ty1.append((r1 * sa) + cy)
            B2L_ty2.append((r2 * sa) + cy)
            B2L_ty3.append((r3 * sa) + cy)
            B2L_ty4.append((r4 * sa) + cy)
            B2L_ty5.append((r5 * sa) + cy)
            B2L_ty6.append((r6 * sa) + cy)

            B2L2_tx0.append(cx - (rr0 * ca))
            B2L2_tx1.append(cx - (rr1 * ca))
            B2L2_tx2.append(cx - (rr2 * ca))
            B2L2_tx3.append(cx - (rr3 * ca))
            B2L2_tx4.append(cx - (rr4 * ca))
            B2L2_tx5.append(cx - (rr5 * ca))
            B2L2_tx6.append(cx - (rr6 * ca))
            B2L2_ty0.append((rr0 * sa) + cy)
            B2L2_ty1.append((rr1 * sa) + cy)
            B2L2_ty2.append((rr2 * sa) + cy)
            B2L2_ty3.append((rr3 * sa) + cy)
            B2L2_ty4.append((rr4 * sa) + cy)
            B2L2_ty5.append((rr5 * sa) + cy)
            B2L2_ty6.append((rr6 * sa) + cy)

        # The rectangles of the middle-left display bar.
        self.ML_rects = []
        self.ML_rects.append([
            loc_ML_0[0], loc_ML_0[1], loc_ML_0[0], loc_ML_0[3],
            loc_ML_0[2], loc_ML_0[1], loc_ML_0[2], loc_ML_0[3],
            loc_ML_0[0], loc_ML_0[1], loc_ML_0[2], loc_ML_0[1],
            loc_ML_0[0], loc_ML_0[3], loc_ML_0[2], loc_ML_0[3],
            ])
        self.ML_rects.append([
            loc_ML_1[0], loc_ML_1[1], loc_ML_1[0], loc_ML_1[3],
            loc_ML_1[2], loc_ML_1[1], loc_ML_1[2], loc_ML_1[3],
            loc_ML_1[0], loc_ML_1[1], loc_ML_1[2], loc_ML_1[1],
            loc_ML_1[0], loc_ML_1[3], loc_ML_1[2], loc_ML_1[3],
            ])
        self.ML_rects.append([
            loc_ML_2[0], loc_ML_2[1], loc_ML_2[0], loc_ML_2[3],
            loc_ML_2[2], loc_ML_2[1], loc_ML_2[2], loc_ML_2[3],
            loc_ML_2[0], loc_ML_2[1], loc_ML_2[2], loc_ML_2[1],
            loc_ML_2[0], loc_ML_2[3], loc_ML_2[2], loc_ML_2[3],
            ])
        self.ML_rects.append([
            loc_ML_3[0], loc_ML_3[1], loc_ML_3[0], loc_ML_3[3],
            loc_ML_3[2], loc_ML_3[1], loc_ML_3[2], loc_ML_3[3],
            loc_ML_3[0], loc_ML_3[1], loc_ML_3[2], loc_ML_3[1],
            loc_ML_3[0], loc_ML_3[3], loc_ML_3[2], loc_ML_3[3],
            ])
        self.ML_rects.append([
            loc_ML_4[0], loc_ML_4[1], loc_ML_4[0], loc_ML_4[3],
            loc_ML_4[2], loc_ML_4[1], loc_ML_4[2], loc_ML_4[3],
            loc_ML_4[0], loc_ML_4[1], loc_ML_4[2], loc_ML_4[1],
            loc_ML_4[0], loc_ML_4[3], loc_ML_4[2], loc_ML_4[3],
            ])
        self.ML_rects.append([
            loc_ML_5[0], loc_ML_5[1], loc_ML_5[0], loc_ML_5[3],
            loc_ML_5[2], loc_ML_5[1], loc_ML_5[2], loc_ML_5[3],
            loc_ML_5[0], loc_ML_5[1], loc_ML_5[2], loc_ML_5[1],
            loc_ML_5[0], loc_ML_5[3], loc_ML_5[2], loc_ML_5[3],
            ])
        self.ML_rects.append([
            loc_ML_6[0], loc_ML_6[1], loc_ML_6[0], loc_ML_6[3],
            loc_ML_6[2], loc_ML_6[1], loc_ML_6[2], loc_ML_6[3],
            loc_ML_6[0], loc_ML_6[1], loc_ML_6[2], loc_ML_6[1],
            loc_ML_6[0], loc_ML_6[3], loc_ML_6[2], loc_ML_6[3],
            ])

        self.ML2_rects = []
        self.ML2_rects.append([
            loc_ML2_0[0], loc_ML2_0[1], loc_ML2_0[0], loc_ML2_0[3],
            loc_ML2_0[2], loc_ML2_0[1], loc_ML2_0[2], loc_ML2_0[3],
            loc_ML2_0[0], loc_ML2_0[1], loc_ML2_0[2], loc_ML2_0[1],
            loc_ML2_0[0], loc_ML2_0[3], loc_ML2_0[2], loc_ML2_0[3],
            ])
        self.ML2_rects.append([
            loc_ML2_1[0], loc_ML2_1[1], loc_ML2_1[0], loc_ML2_1[3],
            loc_ML2_1[2], loc_ML2_1[1], loc_ML2_1[2], loc_ML2_1[3],
            loc_ML2_1[0], loc_ML2_1[1], loc_ML2_1[2], loc_ML2_1[1],
            loc_ML2_1[0], loc_ML2_1[3], loc_ML2_1[2], loc_ML2_1[3],
            ])
        self.ML2_rects.append([
            loc_ML2_2[0], loc_ML2_2[1], loc_ML2_2[0], loc_ML2_2[3],
            loc_ML2_2[2], loc_ML2_2[1], loc_ML2_2[2], loc_ML2_2[3],
            loc_ML2_2[0], loc_ML2_2[1], loc_ML2_2[2], loc_ML2_2[1],
            loc_ML2_2[0], loc_ML2_2[3], loc_ML2_2[2], loc_ML2_2[3],
            ])
        self.ML2_rects.append([
            loc_ML2_3[0], loc_ML2_3[1], loc_ML2_3[0], loc_ML2_3[3],
            loc_ML2_3[2], loc_ML2_3[1], loc_ML2_3[2], loc_ML2_3[3],
            loc_ML2_3[0], loc_ML2_3[1], loc_ML2_3[2], loc_ML2_3[1],
            loc_ML2_3[0], loc_ML2_3[3], loc_ML2_3[2], loc_ML2_3[3],
            ])
        self.ML2_rects.append([
            loc_ML2_4[0], loc_ML2_4[1], loc_ML2_4[0], loc_ML2_4[3],
            loc_ML2_4[2], loc_ML2_4[1], loc_ML2_4[2], loc_ML2_4[3],
            loc_ML2_4[0], loc_ML2_4[1], loc_ML2_4[2], loc_ML2_4[1],
            loc_ML2_4[0], loc_ML2_4[3], loc_ML2_4[2], loc_ML2_4[3],
            ])
        self.ML2_rects.append([
            loc_ML2_5[0], loc_ML2_5[1], loc_ML2_5[0], loc_ML2_5[3],
            loc_ML2_5[2], loc_ML2_5[1], loc_ML2_5[2], loc_ML2_5[3],
            loc_ML2_5[0], loc_ML2_5[1], loc_ML2_5[2], loc_ML2_5[1],
            loc_ML2_5[0], loc_ML2_5[3], loc_ML2_5[2], loc_ML2_5[3],
            ])
        self.ML2_rects.append([
            loc_ML2_6[0], loc_ML2_6[1], loc_ML2_6[0], loc_ML2_6[3],
            loc_ML2_6[2], loc_ML2_6[1], loc_ML2_6[2], loc_ML2_6[3],
            loc_ML2_6[0], loc_ML2_6[1], loc_ML2_6[2], loc_ML2_6[1],
            loc_ML2_6[0], loc_ML2_6[3], loc_ML2_6[2], loc_ML2_6[3],
            ])

        # The rectangles of the middle-right display bar.
        self.MR_rects = []
        self.MR_rects.append([
            loc_MR_0[0], loc_MR_0[1], loc_MR_0[0], loc_MR_0[3],
            loc_MR_0[2], loc_MR_0[1], loc_MR_0[2], loc_MR_0[3],
            loc_MR_0[0], loc_MR_0[1], loc_MR_0[2], loc_MR_0[1],
            loc_MR_0[0], loc_MR_0[3], loc_MR_0[2], loc_MR_0[3],
            ])
        self.MR_rects.append([
            loc_MR_1[0], loc_MR_1[1], loc_MR_1[0], loc_MR_1[3],
            loc_MR_1[2], loc_MR_1[1], loc_MR_1[2], loc_MR_1[3],
            loc_MR_1[0], loc_MR_1[1], loc_MR_1[2], loc_MR_1[1],
            loc_MR_1[0], loc_MR_1[3], loc_MR_1[2], loc_MR_1[3],
            ])
        self.MR_rects.append([
            loc_MR_2[0], loc_MR_2[1], loc_MR_2[0], loc_MR_2[3],
            loc_MR_2[2], loc_MR_2[1], loc_MR_2[2], loc_MR_2[3],
            loc_MR_2[0], loc_MR_2[1], loc_MR_2[2], loc_MR_2[1],
            loc_MR_2[0], loc_MR_2[3], loc_MR_2[2], loc_MR_2[3],
            ])
        self.MR_rects.append([
            loc_MR_3[0], loc_MR_3[1], loc_MR_3[0], loc_MR_3[3],
            loc_MR_3[2], loc_MR_3[1], loc_MR_3[2], loc_MR_3[3],
            loc_MR_3[0], loc_MR_3[1], loc_MR_3[2], loc_MR_3[1],
            loc_MR_3[0], loc_MR_3[3], loc_MR_3[2], loc_MR_3[3],
            ])
        self.MR_rects.append([
            loc_MR_4[0], loc_MR_4[1], loc_MR_4[0], loc_MR_4[3],
            loc_MR_4[2], loc_MR_4[1], loc_MR_4[2], loc_MR_4[3],
            loc_MR_4[0], loc_MR_4[1], loc_MR_4[2], loc_MR_4[1],
            loc_MR_4[0], loc_MR_4[3], loc_MR_4[2], loc_MR_4[3],
            ])
        self.MR_rects.append([
            loc_MR_5[0], loc_MR_5[1], loc_MR_5[0], loc_MR_5[3],
            loc_MR_5[2], loc_MR_5[1], loc_MR_5[2], loc_MR_5[3],
            loc_MR_5[0], loc_MR_5[1], loc_MR_5[2], loc_MR_5[1],
            loc_MR_5[0], loc_MR_5[3], loc_MR_5[2], loc_MR_5[3],
            ])
        self.MR_rects.append([
            loc_MR_6[0], loc_MR_6[1], loc_MR_6[0], loc_MR_6[3],
            loc_MR_6[2], loc_MR_6[1], loc_MR_6[2], loc_MR_6[3],
            loc_MR_6[0], loc_MR_6[1], loc_MR_6[2], loc_MR_6[1],
            loc_MR_6[0], loc_MR_6[3], loc_MR_6[2], loc_MR_6[3],
            ])

        self.MR2_rects = []
        self.MR2_rects.append([
            loc_MR2_0[0], loc_MR2_0[1], loc_MR2_0[0], loc_MR2_0[3],
            loc_MR2_0[2], loc_MR2_0[1], loc_MR2_0[2], loc_MR2_0[3],
            loc_MR2_0[0], loc_MR2_0[1], loc_MR2_0[2], loc_MR2_0[1],
            loc_MR2_0[0], loc_MR2_0[3], loc_MR2_0[2], loc_MR2_0[3],
            ])
        self.MR2_rects.append([
            loc_MR2_1[0], loc_MR2_1[1], loc_MR2_1[0], loc_MR2_1[3],
            loc_MR2_1[2], loc_MR2_1[1], loc_MR2_1[2], loc_MR2_1[3],
            loc_MR2_1[0], loc_MR2_1[1], loc_MR2_1[2], loc_MR2_1[1],
            loc_MR2_1[0], loc_MR2_1[3], loc_MR2_1[2], loc_MR2_1[3],
            ])
        self.MR2_rects.append([
            loc_MR2_2[0], loc_MR2_2[1], loc_MR2_2[0], loc_MR2_2[3],
            loc_MR2_2[2], loc_MR2_2[1], loc_MR2_2[2], loc_MR2_2[3],
            loc_MR2_2[0], loc_MR2_2[1], loc_MR2_2[2], loc_MR2_2[1],
            loc_MR2_2[0], loc_MR2_2[3], loc_MR2_2[2], loc_MR2_2[3],
            ])
        self.MR2_rects.append([
            loc_MR2_3[0], loc_MR2_3[1], loc_MR2_3[0], loc_MR2_3[3],
            loc_MR2_3[2], loc_MR2_3[1], loc_MR2_3[2], loc_MR2_3[3],
            loc_MR2_3[0], loc_MR2_3[1], loc_MR2_3[2], loc_MR2_3[1],
            loc_MR2_3[0], loc_MR2_3[3], loc_MR2_3[2], loc_MR2_3[3],
            ])
        self.MR2_rects.append([
            loc_MR2_4[0], loc_MR2_4[1], loc_MR2_4[0], loc_MR2_4[3],
            loc_MR2_4[2], loc_MR2_4[1], loc_MR2_4[2], loc_MR2_4[3],
            loc_MR2_4[0], loc_MR2_4[1], loc_MR2_4[2], loc_MR2_4[1],
            loc_MR2_4[0], loc_MR2_4[3], loc_MR2_4[2], loc_MR2_4[3],
            ])
        self.MR2_rects.append([
            loc_MR2_5[0], loc_MR2_5[1], loc_MR2_5[0], loc_MR2_5[3],
            loc_MR2_5[2], loc_MR2_5[1], loc_MR2_5[2], loc_MR2_5[3],
            loc_MR2_5[0], loc_MR2_5[1], loc_MR2_5[2], loc_MR2_5[1],
            loc_MR2_5[0], loc_MR2_5[3], loc_MR2_5[2], loc_MR2_5[3],
            ])
        self.MR2_rects.append([
            loc_MR2_6[0], loc_MR2_6[1], loc_MR2_6[0], loc_MR2_6[3],
            loc_MR2_6[2], loc_MR2_6[1], loc_MR2_6[2], loc_MR2_6[3],
            loc_MR2_6[0], loc_MR2_6[1], loc_MR2_6[2], loc_MR2_6[1],
            loc_MR2_6[0], loc_MR2_6[3], loc_MR2_6[2], loc_MR2_6[3],
            ])

        # The arc rectangles.
        self.FL_rects0, self.FL_rects1, self.FL_rects2, self.FL_rects3, self.FL_rects4, self.FL_rects5, self.FL_rects6 = [], [], [], [], [], [], []
        self.FR_rects0, self.FR_rects1, self.FR_rects2, self.FR_rects3, self.FR_rects4, self.FR_rects5, self.FR_rects6 = [], [], [], [], [], [], []
        self.BL_rects0, self.BL_rects1, self.BL_rects2, self.BL_rects3, self.BL_rects4, self.BL_rects5, self.BL_rects6 = [], [], [], [], [], [], []
        self.BR_rects0, self.BR_rects1, self.BR_rects2, self.BR_rects3, self.BR_rects4, self.BR_rects5, self.BR_rects6 = [], [], [], [], [], [], []
        self.FL2_rects0, self.FL2_rects1, self.FL2_rects2, self.FL2_rects3, self.FL2_rects4, self.FL2_rects5, self.FL2_rects6 = [], [], [], [], [], [], []
        self.FR2_rects0, self.FR2_rects1, self.FR2_rects2, self.FR2_rects3, self.FR2_rects4, self.FR2_rects5, self.FR2_rects6 = [], [], [], [], [], [], []
        self.BL2_rects0, self.BL2_rects1, self.BL2_rects2, self.BL2_rects3, self.BL2_rects4, self.BL2_rects5, self.BL2_rects6 = [], [], [], [], [], [], []
        self.BR2_rects0, self.BR2_rects1, self.BR2_rects2, self.BR2_rects3, self.BR2_rects4, self.BR2_rects5, self.BR2_rects6 = [], [], [], [], [], [], []
        for i in range(div):

            # The front-left (FL) arc rectangles.
            self.FL_rects0.append([
                TR_tx0[i], TR_ty0[i], TR2_tx0[i], TR2_ty0[i],
                TR_tx0[i + 1], TR_ty0[i + 1], TR2_tx0[i + 1], TR2_ty0[i + 1],
                TR_tx0[i], TR_ty0[i], TR_tx0[i + 1], TR_ty0[i + 1],
                TR2_tx0[i], TR2_ty0[i], TR2_tx0[i + 1], TR2_ty0[i + 1] ])
            self.FL_rects1.append([
                TR_tx1[i], TR_ty1[i], TR2_tx1[i], TR2_ty1[i],
                TR_tx1[i + 1], TR_ty1[i + 1], TR2_tx1[i + 1], TR2_ty1[i + 1],
                TR_tx1[i], TR_ty1[i], TR_tx1[i + 1], TR_ty1[i + 1],
                TR2_tx1[i], TR2_ty1[i], TR2_tx1[i + 1], TR2_ty1[i + 1] ])
            self.FL_rects2.append([
                TR_tx2[i], TR_ty2[i], TR2_tx2[i], TR2_ty2[i],
                TR_tx2[i + 1], TR_ty2[i + 1], TR2_tx2[i + 1], TR2_ty2[i + 1],
                TR_tx2[i], TR_ty2[i], TR_tx2[i + 1], TR_ty2[i + 1],
                TR2_tx2[i], TR2_ty2[i], TR2_tx2[i + 1], TR2_ty2[i + 1] ])
            self.FL_rects3.append([
                TR_tx3[i], TR_ty3[i], TR2_tx3[i], TR2_ty3[i],
                TR_tx3[i + 1], TR_ty3[i + 1], TR2_tx3[i + 1], TR2_ty3[i + 1],
                TR_tx3[i], TR_ty3[i], TR_tx3[i + 1], TR_ty3[i + 1],
                TR2_tx3[i], TR2_ty3[i], TR2_tx3[i + 1], TR2_ty3[i + 1] ])
            self.FL_rects4.append([
                TR_tx4[i], TR_ty4[i], TR2_tx4[i], TR2_ty4[i],
                TR_tx4[i + 1], TR_ty4[i + 1], TR2_tx4[i + 1], TR2_ty4[i + 1],
                TR_tx4[i], TR_ty4[i], TR_tx4[i + 1], TR_ty4[i + 1],
                TR2_tx4[i], TR2_ty4[i], TR2_tx4[i + 1], TR2_ty4[i + 1] ])
            self.FL_rects5.append([
                TR_tx5[i], TR_ty5[i], TR2_tx5[i], TR2_ty5[i],
                TR_tx5[i + 1], TR_ty5[i + 1], TR2_tx5[i + 1], TR2_ty5[i + 1],
                TR_tx5[i], TR_ty5[i], TR_tx5[i + 1], TR_ty5[i + 1],
                TR2_tx5[i], TR2_ty5[i], TR2_tx5[i + 1], TR2_ty5[i + 1] ])
            self.FL_rects6.append([
                TR_tx6[i], TR_ty6[i], TR2_tx6[i], TR2_ty6[i],
                TR_tx6[i + 1], TR_ty6[i + 1], TR2_tx6[i + 1], TR2_ty6[i + 1],
                TR_tx6[i], TR_ty6[i], TR_tx6[i + 1], TR_ty6[i + 1],
                TR2_tx6[i], TR2_ty6[i], TR2_tx6[i + 1], TR2_ty6[i + 1] ])

            self.FL2_rects0.append([
                T2R_tx0[i], T2R_ty0[i], T2R2_tx0[i], T2R2_ty0[i],
                T2R_tx0[i + 1], T2R_ty0[i + 1], T2R2_tx0[i + 1], T2R2_ty0[i + 1],
                T2R_tx0[i], T2R_ty0[i], T2R_tx0[i + 1], T2R_ty0[i + 1],
                T2R2_tx0[i], T2R2_ty0[i], T2R2_tx0[i + 1], T2R2_ty0[i + 1] ])
            self.FL2_rects1.append([
                T2R_tx1[i], T2R_ty1[i], T2R2_tx1[i], T2R2_ty1[i],
                T2R_tx1[i + 1], T2R_ty1[i + 1], T2R2_tx1[i + 1], T2R2_ty1[i + 1],
                T2R_tx1[i], T2R_ty1[i], T2R_tx1[i + 1], T2R_ty1[i + 1],
                T2R2_tx1[i], T2R2_ty1[i], T2R2_tx1[i + 1], T2R2_ty1[i + 1] ])
            self.FL2_rects2.append([
                T2R_tx2[i], T2R_ty2[i], T2R2_tx2[i], T2R2_ty2[i],
                T2R_tx2[i + 1], T2R_ty2[i + 1], T2R2_tx2[i + 1], T2R2_ty2[i + 1],
                T2R_tx2[i], T2R_ty2[i], T2R_tx2[i + 1], T2R_ty2[i + 1],
                T2R2_tx2[i], T2R2_ty2[i], T2R2_tx2[i + 1], T2R2_ty2[i + 1] ])
            self.FL2_rects3.append([
                T2R_tx3[i], T2R_ty3[i], T2R2_tx3[i], T2R2_ty3[i],
                T2R_tx3[i + 1], T2R_ty3[i + 1], T2R2_tx3[i + 1], T2R2_ty3[i + 1],
                T2R_tx3[i], T2R_ty3[i], T2R_tx3[i + 1], T2R_ty3[i + 1],
                T2R2_tx3[i], T2R2_ty3[i], T2R2_tx3[i + 1], T2R2_ty3[i + 1] ])
            self.FL2_rects4.append([
                T2R_tx4[i], T2R_ty4[i], T2R2_tx4[i], T2R2_ty4[i],
                T2R_tx4[i + 1], T2R_ty4[i + 1], T2R2_tx4[i + 1], T2R2_ty4[i + 1],
                T2R_tx4[i], T2R_ty4[i], T2R_tx4[i + 1], T2R_ty4[i + 1],
                T2R2_tx4[i], T2R2_ty4[i], T2R2_tx4[i + 1], T2R2_ty4[i + 1] ])
            self.FL2_rects5.append([
                T2R_tx5[i], T2R_ty5[i], T2R2_tx5[i], T2R2_ty5[i],
                T2R_tx5[i + 1], T2R_ty5[i + 1], T2R2_tx5[i + 1], T2R2_ty5[i + 1],
                T2R_tx5[i], T2R_ty5[i], T2R_tx5[i + 1], T2R_ty5[i + 1],
                T2R2_tx5[i], T2R2_ty5[i], T2R2_tx5[i + 1], T2R2_ty5[i + 1] ])
            self.FL2_rects6.append([
                T2R_tx6[i], T2R_ty6[i], T2R2_tx6[i], T2R2_ty6[i],
                T2R_tx6[i + 1], T2R_ty6[i + 1], T2R2_tx6[i + 1], T2R2_ty6[i + 1],
                T2R_tx6[i], T2R_ty6[i], T2R_tx6[i + 1], T2R_ty6[i + 1],
                T2R2_tx6[i], T2R2_ty6[i], T2R2_tx6[i + 1], T2R2_ty6[i + 1] ])

            # The front-right (FR) arc rectangles.
            self.FR_rects0.append([
                BR_tx0[i], BR_ty0[i], BR2_tx0[i], BR2_ty0[i],
                BR_tx0[i + 1], BR_ty0[i + 1], BR2_tx0[i + 1], BR2_ty0[i + 1],
                BR_tx0[i], BR_ty0[i], BR_tx0[i + 1], BR_ty0[i + 1],
                BR2_tx0[i], BR2_ty0[i], BR2_tx0[i + 1], BR2_ty0[i + 1] ])
            self.FR_rects1.append([
                BR_tx1[i], BR_ty1[i], BR2_tx1[i], BR2_ty1[i],
                BR_tx1[i + 1], BR_ty1[i + 1], BR2_tx1[i + 1], BR2_ty1[i + 1],
                BR_tx1[i], BR_ty1[i], BR_tx1[i + 1], BR_ty1[i + 1],
                BR2_tx1[i], BR2_ty1[i], BR2_tx1[i + 1], BR2_ty1[i + 1] ])
            self.FR_rects2.append([
                BR_tx2[i], BR_ty2[i], BR2_tx2[i], BR2_ty2[i],
                BR_tx2[i + 1], BR_ty2[i + 1], BR2_tx2[i + 1], BR2_ty2[i + 1],
                BR_tx2[i], BR_ty2[i], BR_tx2[i + 1], BR_ty2[i + 1],
                BR2_tx2[i], BR2_ty2[i], BR2_tx2[i + 1], BR2_ty2[i + 1] ])
            self.FR_rects3.append([
                BR_tx3[i], BR_ty3[i], BR2_tx3[i], BR2_ty3[i],
                BR_tx3[i + 1], BR_ty3[i + 1], BR2_tx3[i + 1], BR2_ty3[i + 1],
                BR_tx3[i], BR_ty3[i], BR_tx3[i + 1], BR_ty3[i + 1],
                BR2_tx3[i], BR2_ty3[i], BR2_tx3[i + 1], BR2_ty3[i + 1] ])
            self.FR_rects4.append([
                BR_tx4[i], BR_ty4[i], BR2_tx4[i], BR2_ty4[i],
                BR_tx4[i + 1], BR_ty4[i + 1], BR2_tx4[i + 1], BR2_ty4[i + 1],
                BR_tx4[i], BR_ty4[i], BR_tx4[i + 1], BR_ty4[i + 1],
                BR2_tx4[i], BR2_ty4[i], BR2_tx4[i + 1], BR2_ty4[i + 1] ])
            self.FR_rects5.append([
                BR_tx5[i], BR_ty5[i], BR2_tx5[i], BR2_ty5[i],
                BR_tx5[i + 1], BR_ty5[i + 1], BR2_tx5[i + 1], BR2_ty5[i + 1],
                BR_tx5[i], BR_ty5[i], BR_tx5[i + 1], BR_ty5[i + 1],
                BR2_tx5[i], BR2_ty5[i], BR2_tx5[i + 1], BR2_ty5[i + 1] ])
            self.FR_rects6.append([
                BR_tx6[i], BR_ty6[i], BR2_tx6[i], BR2_ty6[i],
                BR_tx6[i + 1], BR_ty6[i + 1], BR2_tx6[i + 1], BR2_ty6[i + 1],
                BR_tx6[i], BR_ty6[i], BR_tx6[i + 1], BR_ty6[i + 1],
                BR2_tx6[i], BR2_ty6[i], BR2_tx6[i + 1], BR2_ty6[i + 1] ])

            self.FR2_rects0.append([
                B2R_tx0[i], B2R_ty0[i], B2R2_tx0[i], B2R2_ty0[i],
                B2R_tx0[i + 1], B2R_ty0[i + 1], B2R2_tx0[i + 1], B2R2_ty0[i + 1],
                B2R_tx0[i], B2R_ty0[i], B2R_tx0[i + 1], B2R_ty0[i + 1],
                B2R2_tx0[i], B2R2_ty0[i], B2R2_tx0[i + 1], B2R2_ty0[i + 1] ])
            self.FR2_rects1.append([
                B2R_tx1[i], B2R_ty1[i], B2R2_tx1[i], B2R2_ty1[i],
                B2R_tx1[i + 1], B2R_ty1[i + 1], B2R2_tx1[i + 1], B2R2_ty1[i + 1],
                B2R_tx1[i], B2R_ty1[i], B2R_tx1[i + 1], B2R_ty1[i + 1],
                B2R2_tx1[i], B2R2_ty1[i], B2R2_tx1[i + 1], B2R2_ty1[i + 1] ])
            self.FR2_rects2.append([
                B2R_tx2[i], B2R_ty2[i], B2R2_tx2[i], B2R2_ty2[i],
                B2R_tx2[i + 1], B2R_ty2[i + 1], B2R2_tx2[i + 1], B2R2_ty2[i + 1],
                B2R_tx2[i], B2R_ty2[i], B2R_tx2[i + 1], B2R_ty2[i + 1],
                B2R2_tx2[i], B2R2_ty2[i], B2R2_tx2[i + 1], B2R2_ty2[i + 1] ])
            self.FR2_rects3.append([
                B2R_tx3[i], B2R_ty3[i], B2R2_tx3[i], B2R2_ty3[i],
                B2R_tx3[i + 1], B2R_ty3[i + 1], B2R2_tx3[i + 1], B2R2_ty3[i + 1],
                B2R_tx3[i], B2R_ty3[i], B2R_tx3[i + 1], B2R_ty3[i + 1],
                B2R2_tx3[i], B2R2_ty3[i], B2R2_tx3[i + 1], B2R2_ty3[i + 1] ])
            self.FR2_rects4.append([
                B2R_tx4[i], B2R_ty4[i], B2R2_tx4[i], B2R2_ty4[i],
                B2R_tx4[i + 1], B2R_ty4[i + 1], B2R2_tx4[i + 1], B2R2_ty4[i + 1],
                B2R_tx4[i], B2R_ty4[i], B2R_tx4[i + 1], B2R_ty4[i + 1],
                B2R2_tx4[i], B2R2_ty4[i], B2R2_tx4[i + 1], B2R2_ty4[i + 1] ])
            self.FR2_rects5.append([
                B2R_tx5[i], B2R_ty5[i], B2R2_tx5[i], B2R2_ty5[i],
                B2R_tx5[i + 1], B2R_ty5[i + 1], B2R2_tx5[i + 1], B2R2_ty5[i + 1],
                B2R_tx5[i], B2R_ty5[i], B2R_tx5[i + 1], B2R_ty5[i + 1],
                B2R2_tx5[i], B2R2_ty5[i], B2R2_tx5[i + 1], B2R2_ty5[i + 1] ])
            self.FR2_rects6.append([
                B2R_tx6[i], B2R_ty6[i], B2R2_tx6[i], B2R2_ty6[i],
                B2R_tx6[i + 1], B2R_ty6[i + 1], B2R2_tx6[i + 1], B2R2_ty6[i + 1],
                B2R_tx6[i], B2R_ty6[i], B2R_tx6[i + 1], B2R_ty6[i + 1],
                B2R2_tx6[i], B2R2_ty6[i], B2R2_tx6[i + 1], B2R2_ty6[i + 1] ])

            # The back-left (BL) arc rectangles.
            self.BL_rects0.append([
                TL_tx0[i], TL_ty0[i], TL2_tx0[i], TL2_ty0[i],
                TL_tx0[i + 1], TL_ty0[i + 1], TL2_tx0[i + 1], TL2_ty0[i + 1],
                TL_tx0[i], TL_ty0[i], TL_tx0[i + 1], TL_ty0[i + 1],
                TL2_tx0[i], TL2_ty0[i], TL2_tx0[i + 1], TL2_ty0[i + 1] ])
            self.BL_rects1.append([
                TL_tx1[i], TL_ty1[i], TL2_tx1[i], TL2_ty1[i],
                TL_tx1[i + 1], TL_ty1[i + 1], TL2_tx1[i + 1], TL2_ty1[i + 1],
                TL_tx1[i], TL_ty1[i], TL_tx1[i + 1], TL_ty1[i + 1],
                TL2_tx1[i], TL2_ty1[i], TL2_tx1[i + 1], TL2_ty1[i + 1] ])
            self.BL_rects2.append([
                TL_tx2[i], TL_ty2[i], TL2_tx2[i], TL2_ty2[i],
                TL_tx2[i + 1], TL_ty2[i + 1], TL2_tx2[i + 1], TL2_ty2[i + 1],
                TL_tx2[i], TL_ty2[i], TL_tx2[i + 1], TL_ty2[i + 1],
                TL2_tx2[i], TL2_ty2[i], TL2_tx2[i + 1], TL2_ty2[i + 1] ])
            self.BL_rects3.append([
                TL_tx3[i], TL_ty3[i], TL2_tx3[i], TL2_ty3[i],
                TL_tx3[i + 1], TL_ty3[i + 1], TL2_tx3[i + 1], TL2_ty3[i + 1],
                TL_tx3[i], TL_ty3[i], TL_tx3[i + 1], TL_ty3[i + 1],
                TL2_tx3[i], TL2_ty3[i], TL2_tx3[i + 1], TL2_ty3[i + 1] ])
            self.BL_rects4.append([
                TL_tx4[i], TL_ty4[i], TL2_tx4[i], TL2_ty4[i],
                TL_tx4[i + 1], TL_ty4[i + 1], TL2_tx4[i + 1], TL2_ty4[i + 1],
                TL_tx4[i], TL_ty4[i], TL_tx4[i + 1], TL_ty4[i + 1],
                TL2_tx4[i], TL2_ty4[i], TL2_tx4[i + 1], TL2_ty4[i + 1] ])
            self.BL_rects5.append([
                TL_tx5[i], TL_ty5[i], TL2_tx5[i], TL2_ty5[i],
                TL_tx5[i + 1], TL_ty5[i + 1], TL2_tx5[i + 1], TL2_ty5[i + 1],
                TL_tx5[i], TL_ty5[i], TL_tx5[i + 1], TL_ty5[i + 1],
                TL2_tx5[i], TL2_ty5[i], TL2_tx5[i + 1], TL2_ty5[i + 1] ])
            self.BL_rects6.append([
                TL_tx6[i], TL_ty6[i], TL2_tx6[i], TL2_ty6[i],
                TL_tx6[i + 1], TL_ty6[i + 1], TL2_tx6[i + 1], TL2_ty6[i + 1],
                TL_tx6[i], TL_ty6[i], TL_tx6[i + 1], TL_ty6[i + 1],
                TL2_tx6[i], TL2_ty6[i], TL2_tx6[i + 1], TL2_ty6[i + 1] ])

            self.BL2_rects0.append([
                T2L_tx0[i], T2L_ty0[i], T2L2_tx0[i], T2L2_ty0[i],
                T2L_tx0[i + 1], T2L_ty0[i + 1], T2L2_tx0[i + 1], T2L2_ty0[i + 1],
                T2L_tx0[i], T2L_ty0[i], T2L_tx0[i + 1], T2L_ty0[i + 1],
                T2L2_tx0[i], T2L2_ty0[i], T2L2_tx0[i + 1], T2L2_ty0[i + 1] ])
            self.BL2_rects1.append([
                T2L_tx1[i], T2L_ty1[i], T2L2_tx1[i], T2L2_ty1[i],
                T2L_tx1[i + 1], T2L_ty1[i + 1], T2L2_tx1[i + 1], T2L2_ty1[i + 1],
                T2L_tx1[i], T2L_ty1[i], T2L_tx1[i + 1], T2L_ty1[i + 1],
                T2L2_tx1[i], T2L2_ty1[i], T2L2_tx1[i + 1], T2L2_ty1[i + 1] ])
            self.BL2_rects2.append([
                T2L_tx2[i], T2L_ty2[i], T2L2_tx2[i], T2L2_ty2[i],
                T2L_tx2[i + 1], T2L_ty2[i + 1], T2L2_tx2[i + 1], T2L2_ty2[i + 1],
                T2L_tx2[i], T2L_ty2[i], T2L_tx2[i + 1], T2L_ty2[i + 1],
                T2L2_tx2[i], T2L2_ty2[i], T2L2_tx2[i + 1], T2L2_ty2[i + 1] ])
            self.BL2_rects3.append([
                T2L_tx3[i], T2L_ty3[i], T2L2_tx3[i], T2L2_ty3[i],
                T2L_tx3[i + 1], T2L_ty3[i + 1], T2L2_tx3[i + 1], T2L2_ty3[i + 1],
                T2L_tx3[i], T2L_ty3[i], T2L_tx3[i + 1], T2L_ty3[i + 1],
                T2L2_tx3[i], T2L2_ty3[i], T2L2_tx3[i + 1], T2L2_ty3[i + 1] ])
            self.BL2_rects4.append([
                T2L_tx4[i], T2L_ty4[i], T2L2_tx4[i], T2L2_ty4[i],
                T2L_tx4[i + 1], T2L_ty4[i + 1], T2L2_tx4[i + 1], T2L2_ty4[i + 1],
                T2L_tx4[i], T2L_ty4[i], T2L_tx4[i + 1], T2L_ty4[i + 1],
                T2L2_tx4[i], T2L2_ty4[i], T2L2_tx4[i + 1], T2L2_ty4[i + 1] ])
            self.BL2_rects5.append([
                T2L_tx5[i], T2L_ty5[i], T2L2_tx5[i], T2L2_ty5[i],
                T2L_tx5[i + 1], T2L_ty5[i + 1], T2L2_tx5[i + 1], T2L2_ty5[i + 1],
                T2L_tx5[i], T2L_ty5[i], T2L_tx5[i + 1], T2L_ty5[i + 1],
                T2L2_tx5[i], T2L2_ty5[i], T2L2_tx5[i + 1], T2L2_ty5[i + 1] ])
            self.BL2_rects6.append([
                T2L_tx6[i], T2L_ty6[i], T2L2_tx6[i], T2L2_ty6[i],
                T2L_tx6[i + 1], T2L_ty6[i + 1], T2L2_tx6[i + 1], T2L2_ty6[i + 1],
                T2L_tx6[i], T2L_ty6[i], T2L_tx6[i + 1], T2L_ty6[i + 1],
                T2L2_tx6[i], T2L2_ty6[i], T2L2_tx6[i + 1], T2L2_ty6[i + 1] ])

            # The back-right (BR) arc rectangles.
            self.BR_rects0.append([
                BL_tx0[i], BL_ty0[i], BL2_tx0[i], BL2_ty0[i],
                BL_tx0[i + 1], BL_ty0[i + 1], BL2_tx0[i + 1], BL2_ty0[i + 1],
                BL_tx0[i], BL_ty0[i], BL_tx0[i + 1], BL_ty0[i + 1],
                BL2_tx0[i], BL2_ty0[i], BL2_tx0[i + 1], BL2_ty0[i + 1] ])
            self.BR_rects1.append([
                BL_tx1[i], BL_ty1[i], BL2_tx1[i], BL2_ty1[i],
                BL_tx1[i + 1], BL_ty1[i + 1], BL2_tx1[i + 1], BL2_ty1[i + 1],
                BL_tx1[i], BL_ty1[i], BL_tx1[i + 1], BL_ty1[i + 1],
                BL2_tx1[i], BL2_ty1[i], BL2_tx1[i + 1], BL2_ty1[i + 1] ])
            self.BR_rects2.append([
                BL_tx2[i], BL_ty2[i], BL2_tx2[i], BL2_ty2[i],
                BL_tx2[i + 1], BL_ty2[i + 1], BL2_tx2[i + 1], BL2_ty2[i + 1],
                BL_tx2[i], BL_ty2[i], BL_tx2[i + 1], BL_ty2[i + 1],
                BL2_tx2[i], BL2_ty2[i], BL2_tx2[i + 1], BL2_ty2[i + 1] ])
            self.BR_rects3.append([
                BL_tx3[i], BL_ty3[i], BL2_tx3[i], BL2_ty3[i],
                BL_tx3[i + 1], BL_ty3[i + 1], BL2_tx3[i + 1], BL2_ty3[i + 1],
                BL_tx3[i], BL_ty3[i], BL_tx3[i + 1], BL_ty3[i + 1],
                BL2_tx3[i], BL2_ty3[i], BL2_tx3[i + 1], BL2_ty3[i + 1] ])
            self.BR_rects4.append([
                BL_tx4[i], BL_ty4[i], BL2_tx4[i], BL2_ty4[i],
                BL_tx4[i + 1], BL_ty4[i + 1], BL2_tx4[i + 1], BL2_ty4[i + 1],
                BL_tx4[i], BL_ty4[i], BL_tx4[i + 1], BL_ty4[i + 1],
                BL2_tx4[i], BL2_ty4[i], BL2_tx4[i + 1], BL2_ty4[i + 1] ])
            self.BR_rects5.append([
                BL_tx5[i], BL_ty5[i], BL2_tx5[i], BL2_ty5[i],
                BL_tx5[i + 1], BL_ty5[i + 1], BL2_tx5[i + 1], BL2_ty5[i + 1],
                BL_tx5[i], BL_ty5[i], BL_tx5[i + 1], BL_ty5[i + 1],
                BL2_tx5[i], BL2_ty5[i], BL2_tx5[i + 1], BL2_ty5[i + 1] ])
            self.BR_rects6.append([
                BL_tx6[i], BL_ty6[i], BL2_tx6[i], BL2_ty6[i],
                BL_tx6[i + 1], BL_ty6[i + 1], BL2_tx6[i + 1], BL2_ty6[i + 1],
                BL_tx6[i], BL_ty6[i], BL_tx6[i + 1], BL_ty6[i + 1],
                BL2_tx6[i], BL2_ty6[i], BL2_tx6[i + 1], BL2_ty6[i + 1] ])

            self.BR2_rects0.append([
                B2L_tx0[i], B2L_ty0[i], B2L2_tx0[i], B2L2_ty0[i],
                B2L_tx0[i + 1], B2L_ty0[i + 1], B2L2_tx0[i + 1], B2L2_ty0[i + 1],
                B2L_tx0[i], B2L_ty0[i], B2L_tx0[i + 1], B2L_ty0[i + 1],
                B2L2_tx0[i], B2L2_ty0[i], B2L2_tx0[i + 1], B2L2_ty0[i + 1] ])
            self.BR2_rects1.append([
                B2L_tx1[i], B2L_ty1[i], B2L2_tx1[i], B2L2_ty1[i],
                B2L_tx1[i + 1], B2L_ty1[i + 1], B2L2_tx1[i + 1], B2L2_ty1[i + 1],
                B2L_tx1[i], B2L_ty1[i], B2L_tx1[i + 1], B2L_ty1[i + 1],
                B2L2_tx1[i], B2L2_ty1[i], B2L2_tx1[i + 1], B2L2_ty1[i + 1] ])
            self.BR2_rects2.append([
                B2L_tx2[i], B2L_ty2[i], B2L2_tx2[i], B2L2_ty2[i],
                B2L_tx2[i + 1], B2L_ty2[i + 1], B2L2_tx2[i + 1], B2L2_ty2[i + 1],
                B2L_tx2[i], B2L_ty2[i], B2L_tx2[i + 1], B2L_ty2[i + 1],
                B2L2_tx2[i], B2L2_ty2[i], B2L2_tx2[i + 1], B2L2_ty2[i + 1] ])
            self.BR2_rects3.append([
                B2L_tx3[i], B2L_ty3[i], B2L2_tx3[i], B2L2_ty3[i],
                B2L_tx3[i + 1], B2L_ty3[i + 1], B2L2_tx3[i + 1], B2L2_ty3[i + 1],
                B2L_tx3[i], B2L_ty3[i], B2L_tx3[i + 1], B2L_ty3[i + 1],
                B2L2_tx3[i], B2L2_ty3[i], B2L2_tx3[i + 1], B2L2_ty3[i + 1] ])
            self.BR2_rects4.append([
                B2L_tx4[i], B2L_ty4[i], B2L2_tx4[i], B2L2_ty4[i],
                B2L_tx4[i + 1], B2L_ty4[i + 1], B2L2_tx4[i + 1], B2L2_ty4[i + 1],
                B2L_tx4[i], B2L_ty4[i], B2L_tx4[i + 1], B2L_ty4[i + 1],
                B2L2_tx4[i], B2L2_ty4[i], B2L2_tx4[i + 1], B2L2_ty4[i + 1] ])
            self.BR2_rects5.append([
                B2L_tx5[i], B2L_ty5[i], B2L2_tx5[i], B2L2_ty5[i],
                B2L_tx5[i + 1], B2L_ty5[i + 1], B2L2_tx5[i + 1], B2L2_ty5[i + 1],
                B2L_tx5[i], B2L_ty5[i], B2L_tx5[i + 1], B2L_ty5[i + 1],
                B2L2_tx5[i], B2L2_ty5[i], B2L2_tx5[i + 1], B2L2_ty5[i + 1] ])
            self.BR2_rects6.append([
                B2L_tx6[i], B2L_ty6[i], B2L2_tx6[i], B2L2_ty6[i],
                B2L_tx6[i + 1], B2L_ty6[i + 1], B2L2_tx6[i + 1], B2L2_ty6[i + 1],
                B2L_tx6[i], B2L_ty6[i], B2L_tx6[i + 1], B2L_ty6[i + 1],
                B2L2_tx6[i], B2L2_ty6[i], B2L2_tx6[i + 1], B2L2_ty6[i + 1] ])

        # Initialize the sensor display bars and default them to their outermost position.
        self.us_bar_FL = 6
        self.us_bar_FR = 6
        self.us_bar_BL = 6
        self.us_bar_BR = 6
        self.us_bar_ML = 6
        self.us_bar_MR = 6
        self.us_bar_FL2 = 6
        self.us_bar_FR2 = 6
        self.us_bar_BL2 = 6
        self.us_bar_BR2 = 6
        self.us_bar_ML2 = 6
        self.us_bar_MR2 = 6

        # Miscellaneous.
        self.is_pause = False

    def pause(self, is_pause):
        """
        Sets whether to pause this US_View instance or not. If paused, no updates will be performed and the data will remain frozen on screen.

        Args:
            is_pause (bool): Pauses the time series instance if True, otherwise sets it to continue updating.
        """
        self.is_pause = is_pause

    def update(self):

        # If we are pausing, do not perform any data updating.
        if self.is_pause == True:
            return

        # Get the latest readings from the Ultrasonic sensors.
        d_FL, d_FR = self.us_FL.stream()[0], self.us_FR.stream()[0]
        d_BL, d_BR = self.us_BL.stream()[0], self.us_BR.stream()[0]
        d_ML, d_MR = self.us_ML.stream()[0], self.us_MR.stream()[0]
        d_FL2, d_FR2 = self.us_FL2.stream()[0], self.us_FR2.stream()[0]
        d_BL2, d_BR2 = self.us_BL2.stream()[0], self.us_BR2.stream()[0]
        d_ML2, d_MR2 = self.us_ML2.stream()[0], self.us_MR2.stream()[0]

        # Compute the position of the front-left (FL) display bar, in range [0, 6]. LEFT
        if d_FL > 5.0:
            self.us_bar_FL = 6
        elif d_FL < 0.5:
            self.us_bar_FL = 0
        else:
            self.us_bar_FL = int(np.floor(d_FL)) + 1

        # Compute the position of the front-right (FR) display bar, in range [0, 6]. LEFT
        if d_FR > 5.0:
            self.us_bar_FR = 6
        elif d_FR < 0.5:
            self.us_bar_FR = 0
        else:
            self.us_bar_FR = int(np.floor(d_FR)) + 1

        # Compute the position of the back-left (BL) display bar, in range [0, 6]. LEFT
        if d_BL > 5.0:
            self.us_bar_BL = 6
        elif d_BL < 0.5:
            self.us_bar_BL = 0
        else:
            self.us_bar_BL = int(np.floor(d_BL)) + 1

        # Compute the position of the back-right (BR) display bar, in range [0, 6]. LEFT
        if d_BR > 5.0:
            self.us_bar_BR = 6
        elif d_BR < 0.5:
            self.us_bar_BR = 0
        else:
            self.us_bar_BR = int(np.floor(d_BR)) + 1

        # Compute the position of the middle-left (ML) display bar, in range [0, 6]. LEFT
        if d_ML > 5.0:
            self.us_bar_ML = 6
        elif d_ML < 0.5:
            self.us_bar_ML = 0
        else:
            self.us_bar_ML = int(np.floor(d_ML)) + 1

        # Compute the position of the middle-right (MR) display bar, in range [0, 6]. LEFT
        if d_MR > 5.0:
            self.us_bar_MR = 6
        elif d_MR < 0.5:
            self.us_bar_MR = 0
        else:
            self.us_bar_MR = int(np.floor(d_MR)) + 1

        # Compute the position of the front-left (FL) display bar, in range [0, 6]. RIGHT
        if d_FL2 > 5.0:
            self.us_bar_FL2 = 6
        elif d_FL2 < 0.5:
            self.us_bar_FL2 = 0
        else:
            self.us_bar_FL2 = int(np.floor(d_FL2)) + 1

        # Compute the position of the front-right (FR) display bar, in range [0, 6]. RIGHT
        if d_FR2 > 5.0:
            self.us_bar_FR2 = 6
        elif d_FR2 < 0.5:
            self.us_bar_FR2 = 0
        else:
            self.us_bar_FR2 = int(np.floor(d_FR2)) + 1

        # Compute the position of the back-left (BL) display bar, in range [0, 6]. RIGHT
        if d_BL2 > 5.0:
            self.us_bar_BL2 = 6
        elif d_BL2 < 0.5:
            self.us_bar_BL2 = 0
        else:
            self.us_bar_BL2 = int(np.floor(d_BL2)) + 1

        # Compute the position of the back-right (BR) display bar, in range [0, 6]. RIGHT
        if d_BR2 > 5.0:
            self.us_bar_BR2 = 6
        elif d_BR2 < 0.5:
            self.us_bar_BR2 = 0
        else:
            self.us_bar_BR2 = int(np.floor(d_BR2)) + 1

        # Compute the position of the middle-left (ML) display bar, in range [0, 6]. RIGHT
        if d_ML2 > 5.0:
            self.us_bar_ML2 = 6
        elif d_ML2 < 0.5:
            self.us_bar_ML2 = 0
        else:
            self.us_bar_ML2 = int(np.floor(d_ML2)) + 1

        # Compute the position of the middle-right (MR) display bar, in range [0, 6]. RIGHT
        if d_MR2 > 5.0:
            self.us_bar_MR2 = 6
        elif d_MR2 < 0.5:
            self.us_bar_MR2 = 0
        else:
            self.us_bar_MR2 = int(np.floor(d_MR2)) + 1

    def display(self):

        rects = { 'grey' : [], 'white' : [], 'yellow' : [], 'red' : [] }

        # Populate the rects structure with the rectangles from the middle-left (ML) display bar.
        if self.us_bar_ML == 0:
            rects['red'].append(self.ML_rects[0])
            rects['grey'].append(self.ML_rects[1])
            rects['grey'].append(self.ML_rects[2])
            rects['grey'].append(self.ML_rects[3])
            rects['grey'].append(self.ML_rects[4])
            rects['grey'].append(self.ML_rects[5])
            rects['grey'].append(self.ML_rects[6])
        elif self.us_bar_ML == 1:
            rects['grey'].append(self.ML_rects[0])
            rects['yellow'].append(self.ML_rects[1])
            rects['grey'].append(self.ML_rects[2])
            rects['grey'].append(self.ML_rects[3])
            rects['grey'].append(self.ML_rects[4])
            rects['grey'].append(self.ML_rects[5])
            rects['grey'].append(self.ML_rects[6])
        elif self.us_bar_ML == 2:
            rects['grey'].append(self.ML_rects[0])
            rects['grey'].append(self.ML_rects[1])
            rects['yellow'].append(self.ML_rects[2])
            rects['grey'].append(self.ML_rects[3])
            rects['grey'].append(self.ML_rects[4])
            rects['grey'].append(self.ML_rects[5])
            rects['grey'].append(self.ML_rects[6])
        elif self.us_bar_ML == 3:
            rects['grey'].append(self.ML_rects[0])
            rects['grey'].append(self.ML_rects[1])
            rects['grey'].append(self.ML_rects[2])
            rects['yellow'].append(self.ML_rects[3])
            rects['grey'].append(self.ML_rects[4])
            rects['grey'].append(self.ML_rects[5])
            rects['grey'].append(self.ML_rects[6])
        elif self.us_bar_ML == 4:
            rects['grey'].append(self.ML_rects[0])
            rects['grey'].append(self.ML_rects[1])
            rects['grey'].append(self.ML_rects[2])
            rects['grey'].append(self.ML_rects[3])
            rects['white'].append(self.ML_rects[4])
            rects['grey'].append(self.ML_rects[5])
            rects['grey'].append(self.ML_rects[6])
        elif self.us_bar_ML == 5:
            rects['grey'].append(self.ML_rects[0])
            rects['grey'].append(self.ML_rects[1])
            rects['grey'].append(self.ML_rects[2])
            rects['grey'].append(self.ML_rects[3])
            rects['grey'].append(self.ML_rects[4])
            rects['white'].append(self.ML_rects[5])
            rects['grey'].append(self.ML_rects[6])
        else:
            rects['grey'].append(self.ML_rects[0])
            rects['grey'].append(self.ML_rects[1])
            rects['grey'].append(self.ML_rects[2])
            rects['grey'].append(self.ML_rects[3])
            rects['grey'].append(self.ML_rects[4])
            rects['grey'].append(self.ML_rects[5])
            rects['white'].append(self.ML_rects[6])

        if self.us_bar_ML2 == 0:
            rects['red'].append(self.ML2_rects[0])
            rects['grey'].append(self.ML2_rects[1])
            rects['grey'].append(self.ML2_rects[2])
            rects['grey'].append(self.ML2_rects[3])
            rects['grey'].append(self.ML2_rects[4])
            rects['grey'].append(self.ML2_rects[5])
            rects['grey'].append(self.ML2_rects[6])
        elif self.us_bar_ML2 == 1:
            rects['grey'].append(self.ML2_rects[0])
            rects['yellow'].append(self.ML2_rects[1])
            rects['grey'].append(self.ML2_rects[2])
            rects['grey'].append(self.ML2_rects[3])
            rects['grey'].append(self.ML2_rects[4])
            rects['grey'].append(self.ML2_rects[5])
            rects['grey'].append(self.ML2_rects[6])
        elif self.us_bar_ML2 == 2:
            rects['grey'].append(self.ML2_rects[0])
            rects['grey'].append(self.ML2_rects[1])
            rects['yellow'].append(self.ML2_rects[2])
            rects['grey'].append(self.ML2_rects[3])
            rects['grey'].append(self.ML2_rects[4])
            rects['grey'].append(self.ML2_rects[5])
            rects['grey'].append(self.ML2_rects[6])
        elif self.us_bar_ML2 == 3:
            rects['grey'].append(self.ML2_rects[0])
            rects['grey'].append(self.ML2_rects[1])
            rects['grey'].append(self.ML2_rects[2])
            rects['yellow'].append(self.ML2_rects[3])
            rects['grey'].append(self.ML2_rects[4])
            rects['grey'].append(self.ML2_rects[5])
            rects['grey'].append(self.ML2_rects[6])
        elif self.us_bar_ML2 == 4:
            rects['grey'].append(self.ML2_rects[0])
            rects['grey'].append(self.ML2_rects[1])
            rects['grey'].append(self.ML2_rects[2])
            rects['grey'].append(self.ML2_rects[3])
            rects['white'].append(self.ML2_rects[4])
            rects['grey'].append(self.ML2_rects[5])
            rects['grey'].append(self.ML2_rects[6])
        elif self.us_bar_ML2 == 5:
            rects['grey'].append(self.ML2_rects[0])
            rects['grey'].append(self.ML2_rects[1])
            rects['grey'].append(self.ML2_rects[2])
            rects['grey'].append(self.ML2_rects[3])
            rects['grey'].append(self.ML2_rects[4])
            rects['white'].append(self.ML2_rects[5])
            rects['grey'].append(self.ML2_rects[6])
        else:
            rects['grey'].append(self.ML2_rects[0])
            rects['grey'].append(self.ML2_rects[1])
            rects['grey'].append(self.ML2_rects[2])
            rects['grey'].append(self.ML2_rects[3])
            rects['grey'].append(self.ML2_rects[4])
            rects['grey'].append(self.ML2_rects[5])
            rects['white'].append(self.ML2_rects[6])

        # Populate the rects structure with the rectangles from the middle-right (MR) display bar.
        if self.us_bar_MR == 0:
            rects['red'].append(self.MR_rects[0])
            rects['grey'].append(self.MR_rects[1])
            rects['grey'].append(self.MR_rects[2])
            rects['grey'].append(self.MR_rects[3])
            rects['grey'].append(self.MR_rects[4])
            rects['grey'].append(self.MR_rects[5])
            rects['grey'].append(self.MR_rects[6])
        elif self.us_bar_MR == 1:
            rects['grey'].append(self.MR_rects[0])
            rects['yellow'].append(self.MR_rects[1])
            rects['grey'].append(self.MR_rects[2])
            rects['grey'].append(self.MR_rects[3])
            rects['grey'].append(self.MR_rects[4])
            rects['grey'].append(self.MR_rects[5])
            rects['grey'].append(self.MR_rects[6])
        elif self.us_bar_MR == 2:
            rects['grey'].append(self.MR_rects[0])
            rects['grey'].append(self.MR_rects[1])
            rects['yellow'].append(self.MR_rects[2])
            rects['grey'].append(self.MR_rects[3])
            rects['grey'].append(self.MR_rects[4])
            rects['grey'].append(self.MR_rects[5])
            rects['grey'].append(self.MR_rects[6])
        elif self.us_bar_MR == 3:
            rects['grey'].append(self.MR_rects[0])
            rects['grey'].append(self.MR_rects[1])
            rects['grey'].append(self.MR_rects[2])
            rects['yellow'].append(self.MR_rects[3])
            rects['grey'].append(self.MR_rects[4])
            rects['grey'].append(self.MR_rects[5])
            rects['grey'].append(self.MR_rects[6])
        elif self.us_bar_MR == 4:
            rects['grey'].append(self.MR_rects[0])
            rects['grey'].append(self.MR_rects[1])
            rects['grey'].append(self.MR_rects[2])
            rects['grey'].append(self.MR_rects[3])
            rects['white'].append(self.MR_rects[4])
            rects['grey'].append(self.MR_rects[5])
            rects['grey'].append(self.MR_rects[6])
        elif self.us_bar_MR == 5:
            rects['grey'].append(self.MR_rects[0])
            rects['grey'].append(self.MR_rects[1])
            rects['grey'].append(self.MR_rects[2])
            rects['grey'].append(self.MR_rects[3])
            rects['grey'].append(self.MR_rects[4])
            rects['white'].append(self.MR_rects[5])
            rects['grey'].append(self.MR_rects[6])
        else:
            rects['grey'].append(self.MR_rects[0])
            rects['grey'].append(self.MR_rects[1])
            rects['grey'].append(self.MR_rects[2])
            rects['grey'].append(self.MR_rects[3])
            rects['grey'].append(self.MR_rects[4])
            rects['grey'].append(self.MR_rects[5])
            rects['white'].append(self.MR_rects[6])

        if self.us_bar_MR2 == 0:
            rects['red'].append(self.MR2_rects[0])
            rects['grey'].append(self.MR2_rects[1])
            rects['grey'].append(self.MR2_rects[2])
            rects['grey'].append(self.MR2_rects[3])
            rects['grey'].append(self.MR2_rects[4])
            rects['grey'].append(self.MR2_rects[5])
            rects['grey'].append(self.MR2_rects[6])
        elif self.us_bar_MR2 == 1:
            rects['grey'].append(self.MR2_rects[0])
            rects['yellow'].append(self.MR2_rects[1])
            rects['grey'].append(self.MR2_rects[2])
            rects['grey'].append(self.MR2_rects[3])
            rects['grey'].append(self.MR2_rects[4])
            rects['grey'].append(self.MR2_rects[5])
            rects['grey'].append(self.MR2_rects[6])
        elif self.us_bar_MR2 == 2:
            rects['grey'].append(self.MR2_rects[0])
            rects['grey'].append(self.MR2_rects[1])
            rects['yellow'].append(self.MR2_rects[2])
            rects['grey'].append(self.MR2_rects[3])
            rects['grey'].append(self.MR2_rects[4])
            rects['grey'].append(self.MR2_rects[5])
            rects['grey'].append(self.MR2_rects[6])
        elif self.us_bar_MR2 == 3:
            rects['grey'].append(self.MR2_rects[0])
            rects['grey'].append(self.MR2_rects[1])
            rects['grey'].append(self.MR2_rects[2])
            rects['yellow'].append(self.MR2_rects[3])
            rects['grey'].append(self.MR2_rects[4])
            rects['grey'].append(self.MR2_rects[5])
            rects['grey'].append(self.MR2_rects[6])
        elif self.us_bar_MR2 == 4:
            rects['grey'].append(self.MR2_rects[0])
            rects['grey'].append(self.MR2_rects[1])
            rects['grey'].append(self.MR2_rects[2])
            rects['grey'].append(self.MR2_rects[3])
            rects['white'].append(self.MR2_rects[4])
            rects['grey'].append(self.MR2_rects[5])
            rects['grey'].append(self.MR2_rects[6])
        elif self.us_bar_MR2 == 5:
            rects['grey'].append(self.MR2_rects[0])
            rects['grey'].append(self.MR2_rects[1])
            rects['grey'].append(self.MR2_rects[2])
            rects['grey'].append(self.MR2_rects[3])
            rects['grey'].append(self.MR2_rects[4])
            rects['white'].append(self.MR2_rects[5])
            rects['grey'].append(self.MR2_rects[6])
        else:
            rects['grey'].append(self.MR2_rects[0])
            rects['grey'].append(self.MR2_rects[1])
            rects['grey'].append(self.MR2_rects[2])
            rects['grey'].append(self.MR2_rects[3])
            rects['grey'].append(self.MR2_rects[4])
            rects['grey'].append(self.MR2_rects[5])
            rects['white'].append(self.MR2_rects[6])

        # Populate the rects structure with the rectangles from the front-left (FL) display bar.
        if self.us_bar_FL == 0:
            for r in self.FL_rects0:
                rects['red'].append(r)
            for r in self.FL_rects1 + self.FL_rects2 + self.FL_rects3 + self.FL_rects4 + self.FL_rects5 + self.FL_rects6:
                rects['grey'].append(r)
        elif self.us_bar_FL == 1:
            for r in self.FL_rects1:
                rects['yellow'].append(r)
            for r in self.FL_rects0 + self.FL_rects2 + self.FL_rects3 + self.FL_rects4 + self.FL_rects5 + self.FL_rects6:
                rects['grey'].append(r)
        elif self.us_bar_FL == 2:
            for r in self.FL_rects2:
                rects['yellow'].append(r)
            for r in self.FL_rects0 + self.FL_rects1 + self.FL_rects3 + self.FL_rects4 + self.FL_rects5 + self.FL_rects6:
                rects['grey'].append(r)
        elif self.us_bar_FL == 3:
            for r in self.FL_rects3:
                rects['yellow'].append(r)
            for r in self.FL_rects0 + self.FL_rects1 + self.FL_rects2 + self.FL_rects4 + self.FL_rects5 + self.FL_rects6:
                rects['grey'].append(r)
        elif self.us_bar_FL == 4:
            for r in self.FL_rects4:
                rects['white'].append(r)
            for r in self.FL_rects0 + self.FL_rects1 + self.FL_rects2 + self.FL_rects3 + self.FL_rects5 + self.FL_rects6:
                rects['grey'].append(r)
        elif self.us_bar_FL == 5:
            for r in self.FL_rects5:
                rects['white'].append(r)
            for r in self.FL_rects0 + self.FL_rects1 + self.FL_rects2 + self.FL_rects3 + self.FL_rects4 + self.FL_rects6:
                rects['grey'].append(r)
        else:
            for r in self.FL_rects6:
                rects['white'].append(r)
            for r in self.FL_rects0 + self.FL_rects1 + self.FL_rects2 + self.FL_rects3 + self.FL_rects4 + self.FL_rects5:
                rects['grey'].append(r)

        if self.us_bar_FL2 == 0:
            for r in self.FL2_rects0:
                rects['red'].append(r)
            for r in self.FL2_rects1 + self.FL2_rects2 + self.FL2_rects3 + self.FL2_rects4 + self.FL2_rects5 + self.FL2_rects6:
                rects['grey'].append(r)
        elif self.us_bar_FL2 == 1:
            for r in self.FL2_rects1:
                rects['yellow'].append(r)
            for r in self.FL2_rects0 + self.FL2_rects2 + self.FL2_rects3 + self.FL2_rects4 + self.FL2_rects5 + self.FL2_rects6:
                rects['grey'].append(r)
        elif self.us_bar_FL2 == 2:
            for r in self.FL2_rects2:
                rects['yellow'].append(r)
            for r in self.FL2_rects0 + self.FL2_rects1 + self.FL2_rects3 + self.FL2_rects4 + self.FL2_rects5 + self.FL2_rects6:
                rects['grey'].append(r)
        elif self.us_bar_FL2 == 3:
            for r in self.FL2_rects3:
                rects['yellow'].append(r)
            for r in self.FL2_rects0 + self.FL2_rects1 + self.FL2_rects2 + self.FL2_rects4 + self.FL2_rects5 + self.FL2_rects6:
                rects['grey'].append(r)
        elif self.us_bar_FL2 == 4:
            for r in self.FL2_rects4:
                rects['white'].append(r)
            for r in self.FL2_rects0 + self.FL2_rects1 + self.FL2_rects2 + self.FL2_rects3 + self.FL2_rects5 + self.FL2_rects6:
                rects['grey'].append(r)
        elif self.us_bar_FL2 == 5:
            for r in self.FL2_rects5:
                rects['white'].append(r)
            for r in self.FL2_rects0 + self.FL2_rects1 + self.FL2_rects2 + self.FL2_rects3 + self.FL2_rects4 + self.FL2_rects6:
                rects['grey'].append(r)
        else:
            for r in self.FL2_rects6:
                rects['white'].append(r)
            for r in self.FL2_rects0 + self.FL2_rects1 + self.FL2_rects2 + self.FL2_rects3 + self.FL2_rects4 + self.FL2_rects5:
                rects['grey'].append(r)

        # Populate the rects structure with the rectangles from the front-right (FR) display bar.
        if self.us_bar_FR == 0:
            for r in self.FR_rects0:
                rects['red'].append(r)
            for r in self.FR_rects1 + self.FR_rects2 + self.FR_rects3 + self.FR_rects4 + self.FR_rects5 + self.FR_rects6:
                rects['grey'].append(r)
        elif self.us_bar_FR == 1:
            for r in self.FR_rects1:
                rects['yellow'].append(r)
            for r in self.FR_rects0 + self.FR_rects2 + self.FR_rects3 + self.FR_rects4 + self.FR_rects5 + self.FR_rects6:
                rects['grey'].append(r)
        elif self.us_bar_FR == 2:
            for r in self.FR_rects2:
                rects['yellow'].append(r)
            for r in self.FR_rects0 + self.FR_rects1 + self.FR_rects3 + self.FR_rects4 + self.FR_rects5 + self.FR_rects6:
                rects['grey'].append(r)
        elif self.us_bar_FR == 3:
            for r in self.FR_rects3:
                rects['yellow'].append(r)
            for r in self.FR_rects0 + self.FR_rects1 + self.FR_rects2 + self.FR_rects4 + self.FR_rects5 + self.FR_rects6:
                rects['grey'].append(r)
        elif self.us_bar_FR == 4:
            for r in self.FR_rects4:
                rects['white'].append(r)
            for r in self.FR_rects0 + self.FR_rects1 + self.FR_rects2 + self.FR_rects3 + self.FR_rects5 + self.FR_rects6:
                rects['grey'].append(r)
        elif self.us_bar_FR == 5:
            for r in self.FR_rects5:
                rects['white'].append(r)
            for r in self.FR_rects0 + self.FR_rects1 + self.FR_rects2 + self.FR_rects3 + self.FR_rects4 + self.FR_rects6:
                rects['grey'].append(r)
        else:
            for r in self.FR_rects6:
                rects['white'].append(r)
            for r in self.FR_rects0 + self.FR_rects1 + self.FR_rects2 + self.FR_rects3 + self.FR_rects4 + self.FR_rects5:
                rects['grey'].append(r)

        if self.us_bar_FR2 == 0:
            for r in self.FR2_rects0:
                rects['red'].append(r)
            for r in self.FR2_rects1 + self.FR2_rects2 + self.FR2_rects3 + self.FR2_rects4 + self.FR2_rects5 + self.FR2_rects6:
                rects['grey'].append(r)
        elif self.us_bar_FR2 == 1:
            for r in self.FR2_rects1:
                rects['yellow'].append(r)
            for r in self.FR2_rects0 + self.FR2_rects2 + self.FR2_rects3 + self.FR2_rects4 + self.FR2_rects5 + self.FR2_rects6:
                rects['grey'].append(r)
        elif self.us_bar_FR2 == 2:
            for r in self.FR2_rects2:
                rects['yellow'].append(r)
            for r in self.FR2_rects0 + self.FR2_rects1 + self.FR2_rects3 + self.FR2_rects4 + self.FR2_rects5 + self.FR2_rects6:
                rects['grey'].append(r)
        elif self.us_bar_FR2 == 3:
            for r in self.FR2_rects3:
                rects['yellow'].append(r)
            for r in self.FR2_rects0 + self.FR2_rects1 + self.FR2_rects2 + self.FR2_rects4 + self.FR2_rects5 + self.FR2_rects6:
                rects['grey'].append(r)
        elif self.us_bar_FR2 == 4:
            for r in self.FR2_rects4:
                rects['white'].append(r)
            for r in self.FR2_rects0 + self.FR2_rects1 + self.FR2_rects2 + self.FR2_rects3 + self.FR2_rects5 + self.FR2_rects6:
                rects['grey'].append(r)
        elif self.us_bar_FR2 == 5:
            for r in self.FR2_rects5:
                rects['white'].append(r)
            for r in self.FR2_rects0 + self.FR2_rects1 + self.FR2_rects2 + self.FR2_rects3 + self.FR2_rects4 + self.FR2_rects6:
                rects['grey'].append(r)
        else:
            for r in self.FR2_rects6:
                rects['white'].append(r)
            for r in self.FR2_rects0 + self.FR2_rects1 + self.FR2_rects2 + self.FR2_rects3 + self.FR2_rects4 + self.FR2_rects5:
                rects['grey'].append(r)

        # Populate the rects structure with the rectangles from the back-left (BL) display bar.
        if self.us_bar_BL == 0:
            for r in self.BL_rects0:
                rects['red'].append(r)
            for r in self.BL_rects1 + self.BL_rects2 + self.BL_rects3 + self.BL_rects4 + self.BL_rects5 + self.BL_rects6:
                rects['grey'].append(r)
        elif self.us_bar_BL == 1:
            for r in self.BL_rects1:
                rects['yellow'].append(r)
            for r in self.BL_rects0 + self.BL_rects2 + self.BL_rects3 + self.BL_rects4 + self.BL_rects5 + self.BL_rects6:
                rects['grey'].append(r)
        elif self.us_bar_BL == 2:
            for r in self.BL_rects2:
                rects['yellow'].append(r)
            for r in self.BL_rects0 + self.BL_rects1 + self.BL_rects3 + self.BL_rects4 + self.BL_rects5 + self.BL_rects6:
                rects['grey'].append(r)
        elif self.us_bar_BL == 3:
            for r in self.BL_rects3:
                rects['yellow'].append(r)
            for r in self.BL_rects0 + self.BL_rects1 + self.BL_rects2 + self.BL_rects4 + self.BL_rects5 + self.BL_rects6:
                rects['grey'].append(r)
        elif self.us_bar_BL == 4:
            for r in self.BL_rects4:
                rects['white'].append(r)
            for r in self.BL_rects0 + self.BL_rects1 + self.BL_rects2 + self.BL_rects3 + self.BL_rects5 + self.BL_rects6:
                rects['grey'].append(r)
        elif self.us_bar_BL == 5:
            for r in self.BL_rects5:
                rects['white'].append(r)
            for r in self.BL_rects0 + self.BL_rects1 + self.BL_rects2 + self.BL_rects3 + self.BL_rects4 + self.BL_rects6:
                rects['grey'].append(r)
        else:
            for r in self.BL_rects6:
                rects['white'].append(r)
            for r in self.BL_rects0 + self.BL_rects1 + self.BL_rects2 + self.BL_rects3 + self.BL_rects4 + self.BL_rects5:
                rects['grey'].append(r)

        if self.us_bar_BL2 == 0:
            for r in self.BL2_rects0:
                rects['red'].append(r)
            for r in self.BL2_rects1 + self.BL2_rects2 + self.BL2_rects3 + self.BL2_rects4 + self.BL2_rects5 + self.BL2_rects6:
                rects['grey'].append(r)
        elif self.us_bar_BL2 == 1:
            for r in self.BL2_rects1:
                rects['yellow'].append(r)
            for r in self.BL2_rects0 + self.BL2_rects2 + self.BL2_rects3 + self.BL2_rects4 + self.BL2_rects5 + self.BL2_rects6:
                rects['grey'].append(r)
        elif self.us_bar_BL2 == 2:
            for r in self.BL2_rects2:
                rects['yellow'].append(r)
            for r in self.BL2_rects0 + self.BL2_rects1 + self.BL2_rects3 + self.BL2_rects4 + self.BL2_rects5 + self.BL2_rects6:
                rects['grey'].append(r)
        elif self.us_bar_BL2 == 3:
            for r in self.BL2_rects3:
                rects['yellow'].append(r)
            for r in self.BL2_rects0 + self.BL2_rects1 + self.BL2_rects2 + self.BL2_rects4 + self.BL2_rects5 + self.BL2_rects6:
                rects['grey'].append(r)
        elif self.us_bar_BL2 == 4:
            for r in self.BL2_rects4:
                rects['white'].append(r)
            for r in self.BL2_rects0 + self.BL2_rects1 + self.BL2_rects2 + self.BL2_rects3 + self.BL2_rects5 + self.BL2_rects6:
                rects['grey'].append(r)
        elif self.us_bar_BL2 == 5:
            for r in self.BL2_rects5:
                rects['white'].append(r)
            for r in self.BL2_rects0 + self.BL2_rects1 + self.BL2_rects2 + self.BL2_rects3 + self.BL2_rects4 + self.BL2_rects6:
                rects['grey'].append(r)
        else:
            for r in self.BL2_rects6:
                rects['white'].append(r)
            for r in self.BL2_rects0 + self.BL2_rects1 + self.BL2_rects2 + self.BL2_rects3 + self.BL2_rects4 + self.BL2_rects5:
                rects['grey'].append(r)

        # Populate the rects structure with the rectangles from the back-right (BR) display bar.
        if self.us_bar_BR == 0:
            for r in self.BR_rects0:
                rects['red'].append(r)
            for r in self.BR_rects1 + self.BR_rects2 + self.BR_rects3 + self.BR_rects4 + self.BR_rects5 + self.BR_rects6:
                rects['grey'].append(r)
        elif self.us_bar_BR == 1:
            for r in self.BR_rects1:
                rects['yellow'].append(r)
            for r in self.BR_rects0 + self.BR_rects2 + self.BR_rects3 + self.BR_rects4 + self.BR_rects5 + self.BR_rects6:
                rects['grey'].append(r)
        elif self.us_bar_BR == 2:
            for r in self.BR_rects2:
                rects['yellow'].append(r)
            for r in self.BR_rects0 + self.BR_rects1 + self.BR_rects3 + self.BR_rects4 + self.BR_rects5 + self.BR_rects6:
                rects['grey'].append(r)
        elif self.us_bar_BR == 3:
            for r in self.BR_rects3:
                rects['yellow'].append(r)
            for r in self.BR_rects0 + self.BR_rects1 + self.BR_rects2 + self.BR_rects4 + self.BR_rects5 + self.BR_rects6:
                rects['grey'].append(r)
        elif self.us_bar_BR == 4:
            for r in self.BR_rects4:
                rects['white'].append(r)
            for r in self.BR_rects0 + self.BR_rects1 + self.BR_rects2 + self.BR_rects3 + self.BR_rects5 + self.BR_rects6:
                rects['grey'].append(r)
        elif self.us_bar_BR == 5:
            for r in self.BR_rects5:
                rects['white'].append(r)
            for r in self.BR_rects0 + self.BR_rects1 + self.BR_rects2 + self.BR_rects3 + self.BR_rects4 + self.BR_rects6:
                rects['grey'].append(r)
        else:
            for r in self.BR_rects6:
                rects['white'].append(r)
            for r in self.BR_rects0 + self.BR_rects1 + self.BR_rects2 + self.BR_rects3 + self.BR_rects4 + self.BR_rects5:
                rects['grey'].append(r)

        if self.us_bar_BR2 == 0:
            for r in self.BR2_rects0:
                rects['red'].append(r)
            for r in self.BR2_rects1 + self.BR2_rects2 + self.BR2_rects3 + self.BR2_rects4 + self.BR2_rects5 + self.BR2_rects6:
                rects['grey'].append(r)
        elif self.us_bar_BR2 == 1:
            for r in self.BR2_rects1:
                rects['yellow'].append(r)
            for r in self.BR2_rects0 + self.BR2_rects2 + self.BR2_rects3 + self.BR2_rects4 + self.BR2_rects5 + self.BR2_rects6:
                rects['grey'].append(r)
        elif self.us_bar_BR2 == 2:
            for r in self.BR2_rects2:
                rects['yellow'].append(r)
            for r in self.BR2_rects0 + self.BR2_rects1 + self.BR2_rects3 + self.BR2_rects4 + self.BR2_rects5 + self.BR2_rects6:
                rects['grey'].append(r)
        elif self.us_bar_BR2 == 3:
            for r in self.BR2_rects3:
                rects['yellow'].append(r)
            for r in self.BR2_rects0 + self.BR2_rects1 + self.BR2_rects2 + self.BR2_rects4 + self.BR2_rects5 + self.BR2_rects6:
                rects['grey'].append(r)
        elif self.us_bar_BR2 == 4:
            for r in self.BR2_rects4:
                rects['white'].append(r)
            for r in self.BR2_rects0 + self.BR2_rects1 + self.BR2_rects2 + self.BR2_rects3 + self.BR2_rects5 + self.BR2_rects6:
                rects['grey'].append(r)
        elif self.us_bar_BR2 == 5:
            for r in self.BR2_rects5:
                rects['white'].append(r)
            for r in self.BR2_rects0 + self.BR2_rects1 + self.BR2_rects2 + self.BR2_rects3 + self.BR2_rects4 + self.BR2_rects6:
                rects['grey'].append(r)
        else:
            for r in self.BR2_rects6:
                rects['white'].append(r)
            for r in self.BR2_rects0 + self.BR2_rects1 + self.BR2_rects2 + self.BR2_rects3 + self.BR2_rects4 + self.BR2_rects5:
                rects['grey'].append(r)

        return rects


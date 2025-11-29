package rubikscube;

import java.util.Arrays;

class Cubie {
    static Cubie[] CubeSymmetry = new Cubie[16];

    static Cubie[] moveCube = new Cubie[18];

    static long[] moveCubeSym = new long[18];
    static int[] firstMoveSym = new int[48];

    static int[][] SymMult = new int[16][16];
    static int[][] SymMultInv = new int[16][16];
    static int[][] SymMove = new int[16][18];
    static int[] Sym8Move = new int[8 * 18];
    static int[][] SymMoveUD = new int[16][18];

    static char[] FlipR2S = new char[Coordinates.N_FLIP];
    static char[] TwistR2S = new char[Coordinates.N_TWIST];
    static char[] EPermR2S = new char[Coordinates.N_PERM];
    static char[] FlipS2RF = PhaseSolver.USE_TWIST_FLIP_PRUN ? new char[Coordinates.N_FLIP_SYM * 8] : null;

    static char[] SymStateTwist;
    static char[] SymStateFlip;
    static char[] SymStatePerm;


    static char[] FlipS2R = new char[Coordinates.N_FLIP_SYM];
    static char[] TwistS2R = new char[Coordinates.N_TWIST_SYM];
    static char[] EPermS2R = new char[Coordinates.N_PERM_SYM];
    static byte[] Perm2CombP = new byte[Coordinates.N_PERM_SYM];
    static char[] PermInvEdgeSym = new char[Coordinates.N_PERM_SYM];
    static byte[] MPermInv = new byte[Coordinates.N_MPERM];

    static final int SYM_E2C_MAGIC = 0x00DDDD00;
    static int ESym2CSym(int idx) {
        return idx ^ (SYM_E2C_MAGIC >> ((idx & 0xf) << 1) & 3);
    }


    static Cubie urf1 = new Cubie(2531, 1373, 67026819, 1367);
    static Cubie urf2 = new Cubie(2089, 1906, 322752913, 2040);
    static byte[][] urfMove = new byte[][] {
        {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17},
        {6, 7, 8, 0, 1, 2, 3, 4, 5, 15, 16, 17, 9, 10, 11, 12, 13, 14},
        {3, 4, 5, 6, 7, 8, 0, 1, 2, 12, 13, 14, 15, 16, 17, 9, 10, 11},
        {2, 1, 0, 5, 4, 3, 8, 7, 6, 11, 10, 9, 14, 13, 12, 17, 16, 15},
        {8, 7, 6, 2, 1, 0, 5, 4, 3, 17, 16, 15, 11, 10, 9, 14, 13, 12},
        {5, 4, 3, 8, 7, 6, 2, 1, 0, 14, 13, 12, 17, 16, 15, 11, 10, 9}
    };

    byte[] ca = {0, 1, 2, 3, 4, 5, 6, 7};
    byte[] ea = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22};
    Cubie temps = null;

    Cubie() {
    }

    Cubie(int cperm, int twist, int eperm, int flip) {
        this.setCPerm(cperm);
        this.setTwist(twist);
        CubeMapping.setNPerm(ea, eperm, 12, true);
        this.setFlip(flip);
    }

    Cubie(Cubie c) {
        copy(c);
    }

    void copy(Cubie c) {
        for (int i = 0; i < 8; i++) {
            this.ca[i] = c.ca[i];
        }
        for (int i = 0; i < 12; i++) {
            this.ea[i] = c.ea[i];
        }
    }

    void invCubieCube() {
        if (temps == null) {
            temps = new Cubie();
        }
        for (byte edge = 0; edge < 12; edge++) {
            temps.ea[ea[edge] >> 1] = (byte) (edge << 1 | ea[edge] & 1);
        }
        for (byte corn = 0; corn < 8; corn++) {
            temps.ca[ca[corn] & 0x7] = (byte) (corn | 0x20 >> (ca[corn] >> 3) & 0x18);
        }
        copy(temps);
    }

    static void CornMult(Cubie a, Cubie b, Cubie prod) {
        for (int corn = 0; corn < 8; corn++) {
            int oriA = a.ca[b.ca[corn] & 7] >> 3;
            int oriB = b.ca[corn] >> 3;
            prod.ca[corn] = (byte) (a.ca[b.ca[corn] & 7] & 7 | (oriA + oriB) % 3 << 3);
        }
    }

    static void CornMultFull(Cubie a, Cubie b, Cubie prod) {
        for (int corn = 0; corn < 8; corn++) {
            int oriA = a.ca[b.ca[corn] & 7] >> 3;
            int oriB = b.ca[corn] >> 3;
            int ori = oriA + ((oriA < 3) ? oriB : 6 - oriB);
            ori = ori % 3 + ((oriA < 3) == (oriB < 3) ? 0 : 3);
            prod.ca[corn] = (byte) (a.ca[b.ca[corn] & 7] & 7 | ori << 3);
        }
    }

    static void EdgeMult(Cubie a, Cubie b, Cubie prod) {
        for (int ed = 0; ed < 12; ed++) {
            prod.ea[ed] = (byte) (a.ea[b.ea[ed] >> 1] ^ (b.ea[ed] & 1));
        }
    }


    static void CornerConj(Cubie a, int idx, Cubie b) {
        Cubie sinv = CubeSymmetry[SymMultInv[0][idx]];
        Cubie s = CubeSymmetry[idx];
        for (int corn = 0; corn < 8; corn++) {
            int oriA = sinv.ca[a.ca[s.ca[corn] & 7] & 7] >> 3;
            int oriB = a.ca[s.ca[corn] & 7] >> 3;
            int ori = (oriA < 3) ? oriB : (3 - oriB) % 3;
            b.ca[corn] = (byte) (sinv.ca[a.ca[s.ca[corn] & 7] & 7] & 7 | ori << 3);
        }
    }

    static void EdgeConj(Cubie a, int idx, Cubie b) {
        Cubie sinv = CubeSymmetry[SymMultInv[0][idx]];
        Cubie s = CubeSymmetry[idx];
        for (int ed = 0; ed < 12; ed++) {
            b.ea[ed] = (byte) (sinv.ea[a.ea[s.ea[ed] >> 1] >> 1] ^ (a.ea[s.ea[ed] >> 1] & 1) ^ (s.ea[ed] & 1));
        }
    }

    static int getPermSymInv(int idx, int sym, boolean isCorner) {
        int idxi = PermInvEdgeSym[idx];
        if (isCorner) {
            idxi = ESym2CSym(idxi);
        }
        return idxi & 0xfff0 | SymMult[idxi & 0xf][sym];
    }

    static int getSkipMoves(long ssym) {
        int ret = 0;
        for (int i = 1; (ssym >>= 1) != 0; i++) {
            if ((ssym & 1) == 1) {
                ret |= firstMoveSym[i];
            }
        }
        return ret;
    }

    void URFConjugate() {
        if (temps == null) {
            temps = new Cubie();
        }
        CornMult(urf2, this, temps);
        CornMult(temps, urf1, this);
        EdgeMult(urf2, this, temps);
        EdgeMult(temps, urf1, this);
    }

    int getFlip() {
        int idx = 0;
        for (int i = 0; i < 11; i++) {
            idx = idx << 1 | ea[i] & 1;
        }
        return idx;
    }

    void setFlip(int idx) {
        int parity = 0, val;
        for (int i = 10; i >= 0; i--, idx >>= 1) {
            parity ^= (val = idx & 1);
            ea[i] = (byte) (ea[i] & ~1 | val);
        }
        ea[11] = (byte) (ea[11] & ~1 | parity);
    }

    int getFlipSym() {
        return FlipR2S[getFlip()];
    }

    int getTwist() {
        int idx = 0;
        for (int i = 0; i < 7; i++) {
            idx += (idx << 1) + (ca[i] >> 3);
        }
        return idx;
    }

    void setTwist(int idx) {
        int twst = 15, val;
        for (int i = 6; i >= 0; i--, idx /= 3) {
            twst -= (val = idx % 3);
            ca[i] = (byte) (ca[i] & 0x7 | val << 3);
        }
        ca[7] = (byte) (ca[7] & 0x7 | (twst % 3) << 3);
    }

    int getTwistSym() {
        return TwistR2S[getTwist()];
    }

    int getUDSlice() {
        return 494 - CubeMapping.getComb(ea, 8, true);
    }

    void setUDSlice(int idx) {
        CubeMapping.setComb(ea, 494 - idx, 8, true);
    }

    int getCPerm() {
        return CubeMapping.getNPerm(ca, 8, false);
    }

    void setCPerm(int idx) {
        CubeMapping.setNPerm(ca, idx, 8, false);
    }

    int getCPermSym() {
        return ESym2CSym(EPermR2S[getCPerm()]);
    }

    int getEPerm() {
        return CubeMapping.getNPerm(ea, 8, true);
    }

    void setEPerm(int idx) {
        CubeMapping.setNPerm(ea, idx, 8, true);
    }

    int getEPermSym() {
        return EPermR2S[getEPerm()];
    }

    int getMPerm() {
        return CubeMapping.getNPerm(ea, 12, true) % 24;
    }

    void setMPerm(int idx) {
        CubeMapping.setNPerm(ea, idx, 12, true);
    }

    int getCComb() {
        return CubeMapping.getComb(ca, 0, false);
    }

    void setCComb(int idx) {
        CubeMapping.setComb(ca, idx, 0, false);
    }

    int verify() {
        int sum = 0;
        int edgeMask = 0;
        for (int e = 0; e < 12; e++) {
            edgeMask |= 1 << (ea[e] >> 1);
            sum ^= ea[e] & 1;
        }
        if (edgeMask != 0xfff) {
            return 1;// missing edges
        }
        if (sum != 0) {
            return 1;
        }
        int cornMask = 0;
        sum = 0;
        for (int c = 0; c < 8; c++) {
            cornMask |= 1 << (ca[c] & 7);
            sum += ca[c] >> 3;
        }
        if (cornMask != 0xff) {
            return 1;
        }
        if (sum % 3 != 0) {
            return 1;
        }
        if ((CubeMapping.getNParity(CubeMapping.getNPerm(ea, 12, true), 12) ^ CubeMapping.getNParity(getCPerm(), 8)) != 0) {
            return 1;
        }
        return 0;
    }

    long selfSymmetry() {
        Cubie base = new Cubie(this);  
        Cubie tmp  = new Cubie();

        int baseCPerm = base.getCPermSym() >> 4;
        long symMask = 0L;
        for (int urfIdx = 0; urfIdx < 6; urfIdx++) {
            int currentCPerm = base.getCPermSym() >> 4;
            if (currentCPerm == baseCPerm) {
                for (int s = 0; s < 16; s++) {
                    CornerConj(base, SymMultInv[0][s], tmp);
                    if (!Arrays.equals(tmp.ca, ca)) {
                        continue;
                    }

                    EdgeConj(base, SymMultInv[0][s], tmp);
                    if (!Arrays.equals(tmp.ea, ea)) {
                        continue;
                    }

                    int bitIndex = urfIdx << 4 | s;
                    symMask |= 1L << Math.min(bitIndex, 48);
                }
            }

            base.URFConjugate();
            if (urfIdx % 3 == 2) {
                base.invCubieCube();
            }
        }

        return symMask;
    }


    static void initMove() {
        moveCube[0] = new Cubie(15120, 0, 119750400, 0);
        moveCube[3] = new Cubie(21021, 1494, 323403417, 0);
        moveCube[6] = new Cubie(8064, 1236, 29441808, 550);
        moveCube[9] = new Cubie(9, 0, 5880, 0);
        moveCube[12] = new Cubie(1230, 412, 2949660, 0);
        moveCube[15] = new Cubie(224, 137, 328552, 137);
        for (int a = 0; a < 18; a += 3) {
            for (int p = 0; p < 2; p++) {
                moveCube[a + p + 1] = new Cubie();
                EdgeMult(moveCube[a + p], moveCube[a], moveCube[a + p + 1]);
                CornMult(moveCube[a + p], moveCube[a], moveCube[a + p + 1]);
            }
        }
    }



    static void initSym() {
        Cubie c = new Cubie();
        Cubie d = new Cubie();
        Cubie t;

        Cubie f2 = new Cubie(28783, 0, 259268407, 0);
        Cubie u4 = new Cubie(15138, 0, 119765538, 7);
        Cubie lr2 = new Cubie(5167, 0, 83473207, 0);
        for (int i = 0; i < 8; i++) {
            lr2.ca[i] |= 3 << 3;
        }

        for (int i = 0; i < 16; i++) {
            CubeSymmetry[i] = new Cubie(c);
            CornMultFull(c, u4, d);
            EdgeMult(c, u4, d);
            t = d;  d = c;  c = t;
            if (i % 4 == 3) {
                CornMultFull(c, lr2, d);
                EdgeMult(c, lr2, d);
                t = d;  d = c;  c = t;
            }
            if (i % 8 == 7) {
                CornMultFull(c, f2, d);
                EdgeMult(c, f2, d);
                t = d;  d = c;  c = t;
            }
        }
        for (int i = 0; i < 16; i++) {
            for (int j = 0; j < 16; j++) {
                CornMultFull(CubeSymmetry[i], CubeSymmetry[j], c);
                for (int k = 0; k < 16; k++) {
                    if (Arrays.equals(CubeSymmetry[k].ca, c.ca)) {
                        SymMult[i][j] = k; 
                        SymMultInv[k][j] = i; 
                        break;
                    }
                }
            }
        }
        for (int j = 0; j < 18; j++) {
            for (int s = 0; s < 16; s++) {
                CornerConj(moveCube[j], SymMultInv[0][s], c);
                for (int m = 0; m < 18; m++) {
                    if (Arrays.equals(moveCube[m].ca, c.ca)) {
                        SymMove[s][j] = m;
                        SymMoveUD[s][CubeMapping.std2ud[j]] = CubeMapping.std2ud[m];
                        break;
                    }
                }
                if (s % 2 == 0) {
                    Sym8Move[j << 3 | s >> 1] = SymMove[s][j];
                }
            }
        }

        for (int i = 0; i < 18; i++) {
            moveCubeSym[i] = moveCube[i].selfSymmetry();
            int j = i;
            for (int s = 0; s < 48; s++) {
                if (SymMove[s % 16][j] < i) {
                    firstMoveSym[s] |= 1 << i;
                }
                if (s % 16 == 15) {
                    j = urfMove[2][j];
                }
            }
        }
    }

    static int initSym2Raw(final int N_RAW, char[] Sym2Raw, char[] Raw2Sym, char[] SymState, int coord) {
        Cubie c = new Cubie();
        Cubie d = new Cubie();
        int count = 0, idx = 0;
        int sym_inc = coord >= 2 ? 1 : 2;
        boolean isEdge = coord != 1;

        for (int i = 0; i < N_RAW; i++) {
            if (Raw2Sym[i] != 0) {
                continue;
            }
            switch (coord) {
            case 0: c.setFlip(i); break;
            case 1: c.setTwist(i); break;
            case 2: c.setEPerm(i); break;
            }
            for (int s = 0; s < 16; s += sym_inc) {
                if (isEdge) {
                    EdgeConj(c, s, d);
                } else {
                    CornerConj(c, s, d);
                }
                switch (coord) {
                case 0: idx = d.getFlip();
                    break;
                case 1: idx = d.getTwist();
                    break;
                case 2: idx = d.getEPerm();
                    break;
                }
                if (coord == 0 && PhaseSolver.USE_TWIST_FLIP_PRUN) {
                    FlipS2RF[count << 3 | s >> 1] = (char) idx;
                }
                if (idx == i) {
                    SymState[count] |= 1 << (s / sym_inc);
                }
                int symIdx = (count << 4 | s) / sym_inc;
                Raw2Sym[idx] = (char) symIdx;
            }
            Sym2Raw[count++] = (char) i;
        }
        return count;
    }

    static void initFlipSym2Raw() {initSym2Raw(Coordinates.N_FLIP, FlipS2R, FlipR2S, SymStateFlip = new char[Coordinates.N_FLIP_SYM], 0);
    }

    static void initTwistSym2Raw() {
        initSym2Raw(Coordinates.N_TWIST, TwistS2R, TwistR2S, SymStateTwist = new char[Coordinates.N_TWIST_SYM], 1);
    }

    static void initPermSym2Raw() {
        initSym2Raw(Coordinates.N_PERM, EPermS2R, EPermR2S, SymStatePerm = new char[Coordinates.N_PERM_SYM], 2);
        Cubie cc = new Cubie();
        for (int i = 0; i < Coordinates.N_PERM_SYM; i++) {
            cc.setEPerm(EPermS2R[i]);
            Perm2CombP[i] = (byte) (CubeMapping.getComb(cc.ea, 0, true) + (PhaseSolver.USE_COMBP_PRUN ? CubeMapping.getNParity(EPermS2R[i], 8) * 70 : 0));
            cc.invCubieCube();
            PermInvEdgeSym[i] = (char) cc.getEPermSym();
        }
        for (int i = 0; i < Coordinates.N_MPERM; i++) {
            cc.setMPerm(i);
            cc.invCubieCube();
            MPermInv[i] = (byte) cc.getMPerm();
        }
    }

    static {
        Cubie.initMove();
        Cubie.initSym();
    }
}

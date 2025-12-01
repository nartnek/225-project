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

    static char[] FlipR2S = new char[Constants.N_FLIP];
    static char[] TwistR2S = new char[Constants.N_TWIST];
    static char[] EPermR2S = new char[Constants.N_PERM];
    static char[] FlipS2RF = PhaseSolver.USE_TWIST_FLIP_PRUN ? new char[Constants.N_FLIP_SYM * 8] : null;

    static char[] SymStateTwist;
    static char[] SymStateFlip;
    static char[] SymStatePerm;


    static char[] FlipS2R = new char[Constants.N_FLIP_SYM];
    static char[] TwistS2R = new char[Constants.N_TWIST_SYM];
    static char[] EPermS2R = new char[Constants.N_PERM_SYM];
    static byte[] Perm2CombP = new byte[Constants.N_PERM_SYM];
    static char[] PermInvEdgeSym = new char[Constants.N_PERM_SYM];
    static byte[] MPermInv = new byte[Constants.N_MPERM];

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

    void copy(Cubie source) {
        if (source == null) return;

        System.arraycopy(source.ca, 0, this.ca, 0, 8);
        System.arraycopy(source.ea, 0, this.ea, 0, 12);
    }


    void invCubie() {
        if (temps == null) {
            temps = new Cubie();
        }
        // invert edges
        for (int e = 0; e < 12; e++) {
            int piece = ea[e] >>> 1;
            int ori = ea[e] & 1;
            temps.ea[piece] = (byte) ((e << 1) | ori);
        }

        // invert corners
        for (int c = 0; c < 8; c++) {
            int loc = ca[c] & 7;
            int twist = (ca[c] >>> 3);
            int newTwist = (0x20 >>> twist) & 0x18;
            temps.ca[loc] = (byte) (c | newTwist);
        }

        this.copy(temps);
    }

    static void CornMult(Cubie a, Cubie b, Cubie prod) {
        for (int i = 0; i < 8; i++) {
            int loc = b.ca[i] & 7;
            int aOri = a.ca[loc] >>> 3;
            int bOri = b.ca[i] >>> 3;

            int newOri = (aOri + bOri) % 3;
            prod.ca[i] = (byte) ((a.ca[loc] & 7) | (newOri << 3));
        }
    }

    static void CornMultFull(Cubie a, Cubie b, Cubie prod) {
        for (int i = 0; i < 8; i++) {
            int piece = b.ca[i] & 7;
            int oriA = a.ca[piece] >>> 3;
            int oriB = b.ca[i] >>> 3;

            int sum;
            if (oriA < 3) {
                sum = oriA + oriB;
            } else {
                sum = oriA + (6 - oriB);
            }

            int outTwist = (sum % 3) + ((oriA < 3) == (oriB < 3) ? 0 : 3);
            prod.ca[i] = (byte) ((a.ca[piece] & 7) | (outTwist << 3));
        }
    }

    static void EdgeMult(Cubie a, Cubie b, Cubie prod) {
        for (int i = 0; i < 12; i++) {
            int idx = b.ea[i] >>> 1;
            int flip = (b.ea[i] & 1);

            byte base = a.ea[idx];
            prod.ea[i] = (byte) ((base & ~1) | ((base ^ flip) & 1));
        }
    }


    static void CornerConj(Cubie a, int idx, Cubie b) {
        Cubie sInv = CubeSymmetry[SymMultInv[0][idx]];
        Cubie s = CubeSymmetry[idx];
        for (int i = 0; i < 8; i++) {
            int loc = s.ca[i] & 7;
            int mid = a.ca[loc] & 7;

            int oriA = sInv.ca[mid] >>> 3;
            int oriB = a.ca[loc] >>> 3;

            int finalOri = (oriA < 3) ? oriB : ((3 - oriB) % 3);
            b.ca[i] = (byte) ((sInv.ca[mid] & 7) | (finalOri << 3));
        }
    }

    static void EdgeConj(Cubie a, int idx, Cubie b) {
        Cubie sInv = CubeSymmetry[SymMultInv[0][idx]];
        Cubie s = CubeSymmetry[idx];
        for (int e = 0; e < 12; e++) {
            int mapped = s.ea[e] >>> 1;
            int piece = a.ea[mapped] >>> 1;

            int flip =  (a.ea[mapped] & 1)
                    ^ (s.ea[e] & 1)
                    ^ (sInv.ea[piece] & 1);

            b.ea[e] = (byte) ((sInv.ea[piece] & ~1) | flip);
        }
    }

    static int getPermSymInv(int idx, int sym, boolean isCorner) {
        int inv = PermInvEdgeSym[idx];

        if (isCorner) inv = ESym2CSym(inv);

        int base = inv & 0xFFF0;
        int s = inv & 0xF;

        return base | SymMult[s][sym];
    }

    static int getSkipMoves(long ssym) {
        int flags = 0;
        int pos = 1;

        while ((ssym >>= 1) != 0) {
            if ((ssym & 1L) != 0) {
                flags |= firstMoveSym[pos];
            }
            pos++;
        }
        return flags;
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
        int result = 0;

        for (int pos = 0; pos < 11; pos++) {
            result *= 2;              
            result += (ea[pos] & 1);
        }

        return result;
    }

    void setFlip(int idx) {
        int remaining = idx;
        int checksum = 0;

        for (int k = 10; k >= 0; k--) {
            int bit = remaining & 1;
            remaining >>= 1;

            checksum ^= bit;
            ea[k] = (byte)((ea[k] & ~1) | bit);
        }

        ea[11] = (byte)((ea[11] & ~1) | (checksum & 1));
    }

    int getFlipSym() {
        int i = getFlip();
        return FlipR2S[i];
    }

    int getTwist() {
        int code = 0;

        for (int i = 0; i < 7; i++) {
            int t = (ca[i] >>> 3);   
            code = code * 3 + t;
        }

        return code;
    }

    void setTwist(int idx) {
        int sum = 0;
        int temp = idx;

        for (int i = 6; i >= 0; i--) {
            int twist = temp % 3;
            temp /= 3;

            sum += twist;
            ca[i] = (byte)((ca[i] & 7) | (twist << 3));
        }

        // Last twist 
        int last = (3 - (sum % 3)) % 3;
        ca[7] = (byte)((ca[7] & 7) | (last << 3));
    }

    int getTwistSym() {
        int i = getTwist();
        return TwistR2S[i];
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
        Cubie temp = new Cubie();

        long mask = 0L;
        final int target = base.getCPermSym() >>> 4;

        for (int rot = 0; rot < 6; rot++) {

            int cur = base.getCPermSym() >>> 4;
            if (cur == target) {
                for (int s = 0; s < 16; s++) {

                    int invIdx = SymMultInv[0][s];

                    CornerConj(base, invIdx, temp);
                    if (!Arrays.equals(temp.ca, ca)) continue;

                    EdgeConj(base, invIdx, temp);
                    if (!Arrays.equals(temp.ea, ea)) continue;

                    int bitPos = (rot << 4) | s;
                    mask |= (1L << (bitPos > 48 ? 48 : bitPos));
                }
            }

            base.URFConjugate();
            if ((rot + 1) % 3 == 0) {
                base.invCubie();
            }
        }

        return mask;
    }


    static void initMove() {
        moveCube[0] = new Cubie(15120, 0, 119750400, 0);
        moveCube[3] = new Cubie(21021, 1494, 323403417, 0);
        moveCube[6] = new Cubie(8064, 1236, 29441808, 550);
        moveCube[9] = new Cubie(9, 0, 5880, 0);
        moveCube[12] = new Cubie(1230, 412, 2949660, 0);
        moveCube[15] = new Cubie(224, 137, 328552, 137);
        for (int base = 0; base < 18; base += 3) {
            Cubie m0 = moveCube[base];
            for (int p = 0; p < 2; p++) {
                int target = base + p + 1;
                Cubie prev = moveCube[base + p];
                Cubie out = new Cubie();
                EdgeMult(prev, m0, out);
                CornMult(prev, m0, out);
                moveCube[target] = out;
            }
        }
    }



    static void initSym() {
        Cubie cur  = new Cubie();
        Cubie nxt  = new Cubie();
        Cubie tmp;

        Cubie rotF2 = new Cubie(28783, 0, 259268407, 0);
        Cubie rotU4 = new Cubie(15138, 0, 119765538, 7);
        Cubie flipLR = new Cubie(5167, 0, 83473207, 0);

        for (int i = 0; i < 8; i++) {
            flipLR.ca[i] |= (3 << 3);
        }

        for (int s = 0; s < 16; s++) {
            CubeSymmetry[s] = new Cubie(cur);

            CornMultFull(cur, rotU4, nxt);  EdgeMult(cur, rotU4, nxt);
            tmp = cur; cur = nxt; nxt = tmp;

            if ((s & 3) == 3) {
                CornMultFull(cur, flipLR, nxt);  EdgeMult(cur, flipLR, nxt);
                tmp = cur; cur = nxt; nxt = tmp;
            }
            if ((s & 7) == 7) {
                CornMultFull(cur, rotF2, nxt);   EdgeMult(cur, rotF2, nxt);
                tmp = cur; cur = nxt; nxt = tmp;
            }
        }

        for (int a = 0; a < 16; a++) {
            for (int b = 0; b < 16; b++) {
                CornMultFull(CubeSymmetry[a], CubeSymmetry[b], cur);

                for (int x = 0; x < 16; x++) {
                    if (Arrays.equals(cur.ca, CubeSymmetry[x].ca)) {
                        SymMult[a][b] = x;
                        SymMultInv[x][b] = a;
                        break;
                    }
                }
            }
        }

        for (int mv = 0; mv < 18; mv++) {
            for (int s = 0; s < 16; s++) {

                int inv = SymMultInv[0][s];
                CornerConj(moveCube[mv], inv, cur);

                for (int m2 = 0; m2 < 18; m2++) {
                    if (Arrays.equals(cur.ca, moveCube[m2].ca)) {
                        SymMove[s][mv] = m2;
                        SymMoveUD[s][CubeMapping.std2ud[mv]] =
                            CubeMapping.std2ud[m2];
                        break;
                    }
                }

                if ((s & 1) == 0) {
                    Sym8Move[(mv << 3) | (s >> 1)] = SymMove[s][mv];
                }
            }
        }

        for (int mv = 0; mv < 18; mv++) {
            moveCubeSym[mv] = moveCube[mv].selfSymmetry();

            int curMv = mv;
            for (int s = 0; s < 48; s++) {
                if (SymMove[s & 15][curMv] < mv) {
                    firstMoveSym[s] |= (1 << mv);
                }
                if ((s & 15) == 15) {
                    curMv = urfMove[2][curMv];
                }
            }
        }
    }

    static int initSym2Raw(final int N_RAW, char[] Sym2Raw, char[] Raw2Sym, char[] SymState, int coord) {
        Cubie base = new Cubie();
        Cubie tmp = new Cubie();
        int count = 0, idx = 0;
        int sym_inc = coord >= 2 ? 1 : 2;
        boolean isEdge = coord != 1;

        for (int i = 0; i < N_RAW; i++) {
            if (Raw2Sym[i] != 0) {
                continue;
            }
            switch (coord) {
            case 0: base.setFlip(i); break;
            case 1: base.setTwist(i); break;
            case 2: base.setEPerm(i); break;
            }
            for (int s = 0; s < 16; s += sym_inc) {
                if (isEdge) {
                    EdgeConj(base, s, tmp);
                } else {
                    CornerConj(base, s, tmp);
                }
                switch (coord) {
                case 0: idx = tmp.getFlip();
                    break;
                case 1: idx = tmp.getTwist();
                    break;
                case 2: idx = tmp.getEPerm();
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

    static void initFlipSym2Raw() {initSym2Raw(Constants.N_FLIP, FlipS2R, FlipR2S, SymStateFlip = new char[Constants.N_FLIP_SYM], 0);
    }

    static void initTwistSym2Raw() {
        initSym2Raw(Constants.N_TWIST, TwistS2R, TwistR2S, SymStateTwist = new char[Constants.N_TWIST_SYM], 1);
    }

    static void initPermSym2Raw() {
        initSym2Raw(Constants.N_PERM, EPermS2R, EPermR2S, SymStatePerm = new char[Constants.N_PERM_SYM], 2);
        Cubie cc = new Cubie();
        for (int i = 0; i < Constants.N_PERM_SYM; i++) {
            cc.setEPerm(EPermS2R[i]);
            Perm2CombP[i] = (byte) (CubeMapping.getComb(cc.ea, 0, true) + (PhaseSolver.USE_COMBP_PRUN ? CubeMapping.getNParity(EPermS2R[i], 8) * 70 : 0));
            cc.invCubie();
            PermInvEdgeSym[i] = (char) cc.getEPermSym();
        }
        for (int i = 0; i < Constants.N_MPERM; i++) {
            cc.setMPerm(i);
            cc.invCubie();
            MPermInv[i] = (byte) cc.getMPerm();
        }
    }

    static {
        Cubie.initMove();
        Cubie.initSym();
    }
}

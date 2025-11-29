package rubikscube;

class Coordinates {
    static final int N_MOVES = 18;
    static final int N_MOVES2 = 10;

    static final int N_SLICE = 495;
    static final int N_TWIST = 2187;
    static final int N_TWIST_SYM = 324;
    static final int N_FLIP = 2048;
    static final int N_FLIP_SYM = 336;
    static final int N_PERM = 40320;
    static final int N_PERM_SYM = 2768;
    static final int N_MPERM = 24;
    static final int N_COMB = PhaseSolver.USE_COMBP_PRUN ? 140 : 70;
    static final int P2_PARITY_MOVE = PhaseSolver.USE_COMBP_PRUN ? 0xA5 : 0;



    //phase1
    static char[][] UDSliceMove = new char[N_SLICE][N_MOVES];
    static char[][] TwistMove = new char[N_TWIST_SYM][N_MOVES];
    static char[][] FlipMove = new char[N_FLIP_SYM][N_MOVES];
    static char[][] UDSliceConj = new char[N_SLICE][8];
    static int[] UDSliceTwistPrun = new int[N_SLICE * N_TWIST_SYM / 8 + 1];
    static int[] UDSliceFlipPrun = new int[N_SLICE * N_FLIP_SYM / 8 + 1];
    static int[] TwistFlipPrun = PhaseSolver.USE_TWIST_FLIP_PRUN ? new int[N_FLIP * N_TWIST_SYM / 8 + 1] : null;

    //phase2
    static char[][] CPermMove = new char[N_PERM_SYM][N_MOVES2];
    static char[][] EPermMove = new char[N_PERM_SYM][N_MOVES2];
    static char[][] MPermMove = new char[N_MPERM][N_MOVES2];
    static char[][] MPermConj = new char[N_MPERM][16];
    static char[][] CCombPMove;
    static char[][] CCombPConj = new char[N_COMB][16];
    static int[] MCPermPrun = new int[N_MPERM * N_PERM_SYM / 8 + 1];
    static int[] EPermCCombPPrun = new int[N_COMB * N_PERM_SYM / 8 + 1];


    static int initLevel = 0;

    static synchronized void init(boolean fullInit) {
        if (initLevel == 2 || (initLevel == 1 && !fullInit)) {
            return;
        }

        boolean needMoveTables = (initLevel == 0);

        if (needMoveTables) {
            Cubie.initPermSym2Raw();

            initCPermMove();
            initEPermMove();
            initMPermMoveConj();
            initCombPMoveConj();

            Cubie.initFlipSym2Raw();
            Cubie.initTwistSym2Raw();

            initFlipMove();
            initTwistMove();
            initUDSliceMoveConj();
        }

        //Pruning tables
        initMCPermPrun(fullInit);
        initPermCombPPrun(fullInit);
        initSliceTwistPrun(fullInit);
        initSliceFlipPrun(fullInit);

        if (PhaseSolver.USE_TWIST_FLIP_PRUN) {
            initTwistFlipPrun(fullInit);
        }

        initLevel = fullInit ? 2 : 1;
    }


    static void setPruning(int[] table, int index, int value) {
        table[index >> 3] ^= value << (index << 2); 
    }

    static int getPruning(int[] table, int index) {
        return table[index >> 3] >> (index << 2) & 0xf; 
    }

    static void initUDSliceMoveConj() {
        Cubie src = new Cubie();
        Cubie dst = new Cubie();

        for (int slice = 0; slice < N_SLICE; slice++) {
            src.setUDSlice(slice);

            // quarter-turns 
            for (int m = 0; m < N_MOVES; m += 3) {
                Cubie.EdgeMult(src, Cubie.moveCube[m], dst);
                UDSliceMove[slice][m] = (char) dst.getUDSlice();
            }

            // symmetry conj
            for (int s = 0; s < 16; s += 2) {
                Cubie.EdgeConj(src, Cubie.SymMultInv[0][s], dst);
                UDSliceConj[slice][s >> 1] = (char) dst.getUDSlice();
            }
        }

        for (int slice = 0; slice < N_SLICE; slice++) {
            for (int m = 0; m < N_MOVES; m += 3) {
                int val = UDSliceMove[slice][m];
                for (int k = 1; k < 3; k++) {
                    val = UDSliceMove[val][m];
                    UDSliceMove[slice][m + k] = (char) val;
                }
            }
        }
    }


    static void initFlipMove() {
        Cubie src = new Cubie();
        Cubie dest = new Cubie();
        for (int i = 0; i < N_FLIP_SYM; i++) {
            src.setFlip(Cubie.FlipS2R[i]);
            for (int j = 0; j < N_MOVES; j++) {
                Cubie.EdgeMult(src, Cubie.moveCube[j], dest);
                FlipMove[i][j] = (char) dest.getFlipSym();
            }
        }
    }

    static void initTwistMove() {
        Cubie src = new Cubie();
        Cubie dest = new Cubie();
        for (int i = 0; i < N_TWIST_SYM; i++) {
            src.setTwist(Cubie.TwistS2R[i]);
            for (int j = 0; j < N_MOVES; j++) {
                Cubie.CornMult(src, Cubie.moveCube[j], dest);
                TwistMove[i][j] = (char) dest.getTwistSym();
            }
        }
    }

    static void initCPermMove() {
        Cubie src = new Cubie();
        Cubie dest = new Cubie();
        for (int i = 0; i < N_PERM_SYM; i++) {
            src.setCPerm(Cubie.EPermS2R[i]);
            for (int j = 0; j < N_MOVES2; j++) {
                Cubie.CornMult(src, Cubie.moveCube[CubeMapping.ud2std[j]], dest);
                CPermMove[i][j] = (char) dest.getCPermSym();
            }
        }
    }

    static void initEPermMove() {
        Cubie src = new Cubie();
        Cubie dest = new Cubie();
        for (int i = 0; i < N_PERM_SYM; i++) {
            src.setEPerm(Cubie.EPermS2R[i]);
            for (int j = 0; j < N_MOVES2; j++) {
                Cubie.EdgeMult(src, Cubie.moveCube[CubeMapping.ud2std[j]], dest);
                EPermMove[i][j] = (char) dest.getEPermSym();
            }
        }
    }

    static void initMPermMoveConj() {
        Cubie src = new Cubie();
        Cubie dest = new Cubie();
        for (int i = 0; i < N_MPERM; i++) {
            src.setMPerm(i);
            for (int j = 0; j < N_MOVES2; j++) {
                Cubie.EdgeMult(src, Cubie.moveCube[CubeMapping.ud2std[j]], dest);
                MPermMove[i][j] = (char) dest.getMPerm();
            }
            for (int j = 0; j < 16; j++) {
                Cubie.EdgeConj(src, Cubie.SymMultInv[0][j], dest);
                MPermConj[i][j] = (char) dest.getMPerm();
            }
        }
    }

    static void initCombPMoveConj() {
        Cubie src = new Cubie();
        Cubie dest = new Cubie();
        CCombPMove = new char[N_COMB][N_MOVES2];
        for (int i = 0; i < N_COMB; i++) {
            src.setCComb(i % 70);
            for (int j = 0; j < N_MOVES2; j++) {
                Cubie.CornMult(src, Cubie.moveCube[CubeMapping.ud2std[j]], dest);
                CCombPMove[i][j] = (char) (dest.getCComb() + 70 * ((P2_PARITY_MOVE >> j & 1) ^ (i / 70)));
            }
            for (int j = 0; j < 16; j++) {
                Cubie.CornerConj(src, Cubie.SymMultInv[0][j], dest);
                CCombPConj[i][j] = (char) (dest.getCComb() + 70 * (i / 70));
            }
        }
    }

    static boolean hasZero(int val) {
        return ((val - 0x11111111) & ~val & 0x88888888) != 0;
    }


    static void initRawSymPrun(int[] PrunTable, final char[][] RawMove, final char[][] RawConj, final char[][] SymMove, final char[] SymState, final int PrunFlag, final boolean fullInit) {

        final int SYM_SHIFT = PrunFlag & 0xf;
        final int SYM_E2C_MAGIC = ((PrunFlag >> 4) & 1) == 1 ? Cubie.SYM_E2C_MAGIC : 0x00000000;
        final boolean IS_PHASE2 = ((PrunFlag >> 5) & 1) == 1;
        final int INV_DEPTH = PrunFlag >> 8 & 0xf;
        final int MAX_DEPTH = PrunFlag >> 12 & 0xf;
        final int MIN_DEPTH = PrunFlag >> 16 & 0xf;
        final int SEARCH_DEPTH = fullInit ? MAX_DEPTH : MIN_DEPTH;

        final int SYM_MASK = (1 << SYM_SHIFT) - 1;
        final boolean ISTFP = RawMove == null;
        final int N_RAW = ISTFP ? N_FLIP : RawMove.length;
        final int N_SIZE = N_RAW * SymMove.length;
        final int N_MOVES = IS_PHASE2 ? 10 : 18;
        final int NEXT_AXIS_MAGIC = N_MOVES == 10 ? 0x42 : 0x92492;

        int depth = getPruning(PrunTable, N_SIZE) - 1;
        int done = 0;


        if (depth == -1) {
            for (int i = 0; i < N_SIZE / 8 + 1; i++) {
                PrunTable[i] = 0x11111111;
            }
            setPruning(PrunTable, 0, 0 ^ 1);
            depth = 0;
            done = 1;
        }

        while (depth < SEARCH_DEPTH) {
            int mask = (depth + 1) * 0x11111111 ^ 0xffffffff;
            for (int i = 0; i < PrunTable.length; i++) {
                int val = PrunTable[i] ^ mask;
                val &= val >> 1;
                PrunTable[i] += val & (val >> 2) & 0x11111111;
            }

            boolean inv = depth > INV_DEPTH;
            int select = inv ? (depth + 2) : depth;
            int selArrMask = select * 0x11111111;
            int check = inv ? depth : (depth + 2);
            depth++;
            int xorVal = depth ^ (depth + 1);
            int val = 0;
            for (int i = 0; i < N_SIZE; i++, val >>= 4) {
                if ((i & 7) == 0) {
                    val = PrunTable[i >> 3];
                    if (!hasZero(val ^ selArrMask)) {
                        i += 7;
                        continue;
                    }
                }
                if ((val & 0xf) != select) {
                    continue;
                }
                int raw = i % N_RAW;
                int sym = i / N_RAW;
                int flip = 0, fsym = 0;
                if (ISTFP) {
                    flip = Cubie.FlipR2S[raw];
                    fsym = flip & 7;
                    flip >>= 3;
                }

                for (int m = 0; m < N_MOVES; m++) {
                    int symx = SymMove[sym][m];
                    int rawx;
                    if (ISTFP) {
                        rawx = Cubie.FlipS2RF[
                                   FlipMove[flip][Cubie.Sym8Move[m << 3 | fsym]] ^
                                   fsym ^ (symx & SYM_MASK)];
                    } else {
                        rawx = RawConj[RawMove[raw][m]][symx & SYM_MASK];

                    }
                    symx >>= SYM_SHIFT;
                    int idx = symx * N_RAW + rawx;
                    int prun = getPruning(PrunTable, idx);
                    if (prun != check) {
                        if (prun < depth - 1) {
                            m += NEXT_AXIS_MAGIC >> m & 3;
                        }
                        continue;
                    }
                    done++;
                    if (inv) {
                        setPruning(PrunTable, i, xorVal);
                        break;
                    }
                    setPruning(PrunTable, idx, xorVal);
                    for (int j = 1, symState = SymState[symx]; (symState >>= 1) != 0; j++) {
                        if ((symState & 1) != 1) {
                            continue;
                        }
                        int idxx = symx * N_RAW;
                        if (ISTFP) {
                            idxx += Cubie.FlipS2RF[Cubie.FlipR2S[rawx] ^ j];
                        } else {
                            idxx += RawConj[rawx][j ^ (SYM_E2C_MAGIC >> (j << 1) & 3)];
                        }
                        if (getPruning(PrunTable, idxx) == check) {
                            setPruning(PrunTable, idxx, xorVal);
                            done++;
                        }
                    }
                }
            }
            // System.out.println(String.format("%2d%10d%10f", depth, done, (System.nanoTime() - tt) / 1e6d));
        }
    }

    static void initTwistFlipPrun(boolean fullInit) {
        initRawSymPrun(
            TwistFlipPrun,
            null, null,
            TwistMove, Cubie.SymStateTwist, 0x19603,
            fullInit
        );
    }

    static void initSliceTwistPrun(boolean fullInit) {
        initRawSymPrun(
            UDSliceTwistPrun,
            UDSliceMove, UDSliceConj,
            TwistMove, Cubie.SymStateTwist, 0x69603,
            fullInit
        );
    }

    static void initSliceFlipPrun(boolean fullInit) {
        initRawSymPrun(
            UDSliceFlipPrun,
            UDSliceMove, UDSliceConj,
            FlipMove, Cubie.SymStateFlip, 0x69603,
            fullInit
        );
    }

    static void initMCPermPrun(boolean fullInit) {
        initRawSymPrun(
            MCPermPrun,
            MPermMove, MPermConj,
            CPermMove, Cubie.SymStatePerm, 0x8ea34,
            fullInit
        );
    }

    static void initPermCombPPrun(boolean fullInit) {
        initRawSymPrun(EPermCCombPPrun, CCombPMove, CCombPConj, EPermMove, Cubie.SymStatePerm, 0x7d824, fullInit);
    }


    int twist;
    int tsym;
    int flip;
    int fsym;
    int slice;
    int prun;

    int twistc;
    int flipc;

    void set(Coordinates node) {
        this.twist = node.twist;
        this.tsym = node.tsym;
        this.flip = node.flip;
        this.fsym = node.fsym;
        this.slice = node.slice;
        this.prun = node.prun;

        if (PhaseSolver.USE_CONJ_PRUN) {
            this.twistc = node.twistc;
            this.flipc = node.flipc;
        }
    }

    void calcPruning(boolean isPhase1) {
        prun = Math.max(Math.max(getPruning(UDSliceTwistPrun,twist * N_SLICE + UDSliceConj[slice][tsym]),getPruning(UDSliceFlipPrun,flip * N_SLICE + UDSliceConj[slice][fsym])),
                Math.max(PhaseSolver.USE_CONJ_PRUN ? getPruning(TwistFlipPrun,(twistc >> 3) << 11 | Cubie.FlipS2RF[flipc ^ (twistc & 7)]) : 0,PhaseSolver.USE_TWIST_FLIP_PRUN ? getPruning(TwistFlipPrun,twist << 11 | Cubie.FlipS2RF[flip << 3 | (fsym ^ tsym)]) : 0));
    }

    boolean setWithPrun(Cubie cc, int depth) {
        twist = cc.getTwistSym();
        flip = cc.getFlipSym();
        tsym = twist & 7;
        twist = twist >> 3;

        prun = PhaseSolver.USE_TWIST_FLIP_PRUN ? getPruning(TwistFlipPrun,twist << 11 | Cubie.FlipS2RF[flip ^ tsym]) : 0;
        if (prun > depth) {
            return false;
        }

        fsym = flip & 7;
        flip = flip >> 3;

        slice = cc.getUDSlice();
        prun = Math.max(prun, Math.max(getPruning(UDSliceTwistPrun,twist * N_SLICE + UDSliceConj[slice][tsym]),
                            getPruning(UDSliceFlipPrun,flip * N_SLICE + UDSliceConj[slice][fsym])));
        if (prun > depth) {
            return false;
        }

        if (PhaseSolver.USE_CONJ_PRUN) {
            Cubie pc = new Cubie();
            Cubie.CornerConj(cc, 1, pc);
            Cubie.EdgeConj(cc, 1, pc);
            twistc = pc.getTwistSym();
            flipc = pc.getFlipSym();
            prun = Math.max(prun,getPruning(TwistFlipPrun,(twistc >> 3) << 11 | Cubie.FlipS2RF[flipc ^ (twistc & 7)]));
        }

        return prun <= depth;
    }

    int pruningMovesConj(Coordinates cc, int m) {
        m = Cubie.SymMove[3][m];
        flipc = FlipMove[cc.flipc >> 3][Cubie.Sym8Move[m << 3 | cc.flipc & 7]] ^ (cc.flipc & 7);
        twistc = TwistMove[cc.twistc >> 3][Cubie.Sym8Move[m << 3 | cc.twistc & 7]] ^ (cc.twistc & 7);
        return getPruning(TwistFlipPrun, (twistc >> 3) << 11 | Cubie.FlipS2RF[flipc ^ (twistc & 7)]);
    }


    int pruningMoves(Coordinates cc, int m, boolean isPhase1) {
        slice = UDSliceMove[cc.slice][m];

        flip = FlipMove[cc.flip][Cubie.Sym8Move[m << 3 | cc.fsym]];
        fsym = (flip & 7) ^ cc.fsym;
        flip >>= 3;

        twist = TwistMove[cc.twist][Cubie.Sym8Move[m << 3 | cc.tsym]];
        tsym = (twist & 7) ^ cc.tsym;
        twist >>= 3;

        prun = Math.max(Math.max(getPruning(UDSliceTwistPrun,twist * N_SLICE + UDSliceConj[slice][tsym]),getPruning(UDSliceFlipPrun,flip * N_SLICE + UDSliceConj[slice][fsym])),
                PhaseSolver.USE_TWIST_FLIP_PRUN ? getPruning(TwistFlipPrun,twist << 11 | Cubie.FlipS2RF[flip << 3 | (fsym ^ tsym)]) : 0);
        return prun;
    }

    
}

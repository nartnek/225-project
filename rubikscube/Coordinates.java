package rubikscube;

class Coordinates {
    //0 = nothing
    //1 = partial (phase 1 only)
    //2 = full init
    static int initLevel = 0;

    static char[][] UDSliceMove = new char[Constants.N_SLICE][Constants.N_MOVES_PHASE1];

    static char[][] TwistMove = new char[Constants.N_TWIST_SYM][Constants.N_MOVES_PHASE1];

    static char[][] FlipMove = new char[Constants.N_FLIP_SYM][Constants.N_MOVES_PHASE1];

    static char[][] UDSliceConj = new char[Constants.N_SLICE][8];

    static int[] UDSliceTwistPrun = new int[Constants.N_SLICE * Constants.N_TWIST_SYM / 8 + 1];

    static int[] UDSliceFlipPrun = new int[Constants.N_SLICE * Constants.N_FLIP_SYM / 8 + 1];

    static int[] TwistFlipPrun = PhaseSolver.USE_TWIST_FLIP_PRUN ? new int[Constants.N_FLIP * Constants.N_TWIST_SYM / 8 + 1] : null;


    static char[][] CPermMove = new char[Constants.N_PERM_SYM][Constants.N_MOVES_PHASE2];

    static char[][] EPermMove = new char[Constants.N_PERM_SYM][Constants.N_MOVES_PHASE2];

    static char[][] MPermMove = new char[Constants.N_MPERM][Constants.N_MOVES_PHASE2];

    static char[][] MPermConj = new char[Constants.N_MPERM][16];

    static char[][] CCombPMove;
    static char[][] CCombPConj = new char[Constants.N_COMB][16];

    static int[] MCPermPrun = new int[Constants.N_MPERM * Constants.N_PERM_SYM / 8 + 1];

    static int[] EPermCCombPPrun = new int[Constants.N_COMB * Constants.N_PERM_SYM / 8 + 1];


    static synchronized void init(boolean fullInit) {
        if (initLevel == 2 || (!fullInit && initLevel == 1)) {
            return;
        }

        boolean firstInitialization = (initLevel == 0);

        if (firstInitialization) {
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

        initMCPermPrun(fullInit);
        initPermCombPPrun(fullInit);
        initSliceTwistPrun(fullInit);
        initSliceFlipPrun(fullInit);

        if (PhaseSolver.USE_TWIST_FLIP_PRUN) {
            initTwistFlipPrun(fullInit);
        }

        initLevel = fullInit ? 2 : Math.max(initLevel, 1);
    }


    static void setPruning(int[] table, int index, int value) {
        int wordIdx = index >>> 3;               
        int shift = (index & 7) << 2;           
        int mask = value << shift;
        table[wordIdx] ^= mask;
    }

    static int getPruning(int[] table, int index) {
        int wordIdx = index >>> 3;
        int shift = (index & 7) << 2;
        return (table[wordIdx] >>> shift) & 0xF;
    }


    static void initUDSliceMoveConj() {
        Cubie src = new Cubie();
        Cubie dst = new Cubie();

        for (int slice = 0; slice < Constants.N_SLICE; slice++) {
            src.setUDSlice(slice);

            for (int move = 0; move < Constants.N_MOVES_PHASE1; move += 3) {
                Cubie.EdgeMult(src, Cubie.moveCube[move], dst);
                UDSliceMove[slice][move] = (char) dst.getUDSlice();
            }

            for (int s = 0; s < 16; s += 2) {
                int inv = Cubie.SymMultInv[0][s];
                Cubie.EdgeConj(src, inv, dst);
                UDSliceConj[slice][s >> 1] = (char) dst.getUDSlice();
            }
        }

        // Now fill in power-2 and power-3 moves from repeated quarter-turns
        for (int slice = 0; slice < Constants.N_SLICE; slice++) {
            for (int move = 0; move < Constants.N_MOVES_PHASE1; move += 3) {
                int val = UDSliceMove[slice][move];
                for (int k = 1; k < 3; k++) {
                    val = UDSliceMove[val][move];
                    UDSliceMove[slice][move + k] = (char) val;
                }
            }
        }
    }

    // ---------- Phase 1 move tables: flip / twist ----------

    static void initFlipMove() {
        Cubie src = new Cubie();
        Cubie dst = new Cubie();

        for (int i = 0; i < Constants.N_FLIP_SYM; i++) {
            src.setFlip(Cubie.FlipS2R[i]);
            for (int m = 0; m < Constants.N_MOVES_PHASE1; m++) {
                Cubie.EdgeMult(src, Cubie.moveCube[m], dst);
                FlipMove[i][m] = (char) dst.getFlipSym();
            }
        }
    }

    static void initTwistMove() {
        Cubie src = new Cubie();
        Cubie dst = new Cubie();

        for (int i = 0; i < Constants.N_TWIST_SYM; i++) {
            src.setTwist(Cubie.TwistS2R[i]);
            for (int m = 0; m < Constants.N_MOVES_PHASE1; m++) {
                Cubie.CornMult(src, Cubie.moveCube[m], dst);
                TwistMove[i][m] = (char) dst.getTwistSym();
            }
        }
    }

    // ---------- Phase 2 move tables: corner / edge / M perm ----------

    static void initCPermMove() {
        Cubie src = new Cubie();
        Cubie dst = new Cubie();

        for (int i = 0; i < Constants.N_PERM_SYM; i++) {
            src.setCPerm(Cubie.EPermS2R[i]);
            for (int m = 0; m < Constants.N_MOVES_PHASE2; m++) {
                int stdMove = CubeMapping.ud2std[m];
                Cubie.CornMult(src, Cubie.moveCube[stdMove], dst);
                CPermMove[i][m] = (char) dst.getCPermSym();
            }
        }
    }

    static void initEPermMove() {
        Cubie src = new Cubie();
        Cubie dst = new Cubie();

        for (int i = 0; i < Constants.N_PERM_SYM; i++) {
            src.setEPerm(Cubie.EPermS2R[i]);
            for (int m = 0; m < Constants.N_MOVES_PHASE2; m++) {
                int stdMove = CubeMapping.ud2std[m];
                Cubie.EdgeMult(src, Cubie.moveCube[stdMove], dst);
                EPermMove[i][m] = (char) dst.getEPermSym();
            }
        }
    }

    static void initMPermMoveConj() {
        Cubie src = new Cubie();
        Cubie dst = new Cubie();

        for (int p = 0; p < Constants.N_MPERM; p++) {
            src.setMPerm(p);

            // Move table
            for (int m = 0; m < Constants.N_MOVES_PHASE2; m++) {
                int stdMove = CubeMapping.ud2std[m];
                Cubie.EdgeMult(src, Cubie.moveCube[stdMove], dst);
                MPermMove[p][m] = (char) dst.getMPerm();
            }

            // Conjugation table
            for (int s = 0; s < 16; s++) {
                int inv = Cubie.SymMultInv[0][s];
                Cubie.EdgeConj(src, inv, dst);
                MPermConj[p][s] = (char) dst.getMPerm();
            }
        }
    }

    // ---------- Phase 2 combined corner-comb parity tables ----------

    static void initCombPMoveConj() {
        Cubie src = new Cubie();
        Cubie dst = new Cubie();

        CCombPMove = new char[Constants.N_COMB][Constants.N_MOVES_PHASE2];

        for (int c = 0; c < Constants.N_COMB; c++) {
            int baseIndex = c % 70;
            int parity = c / 70;

            src.setCComb(baseIndex);

            // Move table
            for (int m = 0; m < Constants.N_MOVES_PHASE2; m++) {
                int stdMove = CubeMapping.ud2std[m];
                Cubie.CornMult(src, Cubie.moveCube[stdMove], dst);

                int comb = dst.getCComb();
                int flipParity = (Constants.P2_PARITY_MOVE >>> m) & 1;
                int fullIndex = comb + 70 * (flipParity ^ parity);

                CCombPMove[c][m] = (char) fullIndex;
            }

            // Conjugation table
            for (int s = 0; s < 16; s++) {
                int inv = Cubie.SymMultInv[0][s];
                Cubie.CornerConj(src, inv, dst);

                int comb = dst.getCComb();
                int fullIndex = comb + 70 * parity;
                CCombPConj[c][s] = (char) fullIndex;
            }
        }
    }

    // ---------- Utility: detect any zero nibble ----------

    static boolean hasZero(int val) {
        int tmp = (val - Constants.PRUN_UNIT)
                & ~val
                & Constants.PRUN_ZERO_MASK;
        return tmp != 0;
    }

    // ---------- Generic raw/sym pruning builder ----------

    static void initRawSymPrun(
            int[] prunTable,
            final char[][] rawMove,
            final char[][] rawConj,
            final char[][] symMove,
            final char[] symState,
            final int prunFlag,
            final boolean fullInit
    ) {
        // Decode flag structure
        final int symShift    =  prunFlag        & 0xF;
        final boolean e2cFlag = (prunFlag >> 4 & 1) == 1;
        final boolean phase2  = (prunFlag >> 5 & 1) == 1;
        final int invDepth    = (prunFlag >> 8)  & 0xF;
        final int maxDepth    = (prunFlag >> 12) & 0xF;
        final int minDepth    = (prunFlag >> 16) & 0xF;

        final int searchDepth = fullInit ? maxDepth : minDepth;

        final int symMask = (1 << symShift) - 1;
        final boolean twistFlipOnly = (rawMove == null);

        final int rawCount = twistFlipOnly
                ? Constants.N_FLIP
                : rawMove.length;
        final int symCount = symMove.length;
        final int totalStates = rawCount * symCount;

        final int moveCount = phase2
                ? Constants.N_MOVES_PHASE2
                : Constants.N_MOVES_PHASE1;

        final int skipMagic =
                phase2 ? Constants.SKIP_MAGIC_PHASE2
                       : Constants.SKIP_MAGIC_PHASE1;

        final int e2cMagic =
                e2cFlag ? Cubie.SYM_E2C_MAGIC : 0x00000000;

        // Determine current depth from the "sentinel" entry
        int depth = getPruning(prunTable, totalStates) - 1;

        if (depth == -1) {
            // Fresh table: fill with PRUN_UNIT "infinity"
            int words = totalStates / 8 + 1;
            for (int i = 0; i < words; i++) {
                prunTable[i] = Constants.PRUN_UNIT;
            }
            // Mark solved state (0) with distance 0^1 = 1 (special encoding)
            setPruning(prunTable, 0, 0 ^ 1);
            depth = 0;
        }

        while (depth < searchDepth) {
            // "Advance" the frontier by one layer in all words
            int targetDepth = depth + 1;
            int xorMask = (targetDepth * Constants.PRUN_UNIT) ^ 0xFFFFFFFF;

            for (int i = 0; i < prunTable.length; i++) {
                int v = prunTable[i] ^ xorMask;
                v &= v >>> 1;
                prunTable[i] += v & (v >>> 2) & Constants.PRUN_UNIT;
            }

            boolean inverseSearch = depth > invDepth;

            int selectValue = inverseSearch ? (depth + 2) : depth;
            int selectMask = selectValue * Constants.PRUN_UNIT;

            int checkValue = inverseSearch ? depth : (depth + 2);

            depth++;
            int xorValue = depth ^ (depth + 1);

            int word = 0;

            for (int stateIndex = 0; stateIndex < totalStates; stateIndex++, word >>>= 4) {

                if ((stateIndex & 7) == 0) {
                    word = prunTable[stateIndex >>> 3];

                    if (!hasZero(word ^ selectMask)) {
                        stateIndex += 7;
                        continue;
                    }
                }

                if ((word & 0xF) != selectValue) {
                    continue;
                }

                int raw = stateIndex % rawCount;
                int sym = stateIndex / rawCount;

                int flipState = 0;
                int flipSymPart = 0;

                if (twistFlipOnly) {
                    int flipSym = Cubie.FlipR2S[raw];
                    flipSymPart = flipSym & 7;
                    flipState = flipSym >>> 3;
                }

                for (int move = 0; move < moveCount; move++) {
                    int symX = symMove[sym][move];
                    int rawX;

                    if (twistFlipOnly) {
                        int idx1 = Cubie.Sym8Move[(move << 3) | flipSymPart];
                        char flipNext = FlipMove[flipState][idx1];
                        int xorPart = flipSymPart ^ (symX & symMask);
                        int combined = flipNext ^ xorPart;
                        rawX = Cubie.FlipS2RF[combined];
                    } else {
                        int conjSym = symX & symMask;
                        rawX = rawConj[rawMove[raw][move]][conjSym];
                    }

                    symX >>>= symShift;
                    int idx = symX * rawCount + rawX;

                    int prVal = getPruning(prunTable, idx);
                    if (prVal != checkValue) {
                        if (prVal < depth - 1) {
                            move += (skipMagic >>> move) & 3;
                        }
                        continue;
                    }

                    if (inverseSearch) {
                        setPruning(prunTable, stateIndex, xorValue);
                    } else {
                        setPruning(prunTable, idx, xorValue);
                    }

                    // fill all symmetric states for this representative
                    int symStateBits = symState[symX];

                    for (int s = 1; (symStateBits >>>= 1) != 0; s++) {
                        if ((symStateBits & 1) == 0) {
                            continue;
                        }

                        int idxSym = symX * rawCount;

                        if (twistFlipOnly) {
                            int baseFlipSym = Cubie.FlipR2S[rawX];
                            idxSym += Cubie.FlipS2RF[baseFlipSym ^ s];
                        } else {
                            int shift = (e2cMagic >>> (s << 1)) & 3;
                            idxSym += rawConj[rawX][s ^ shift];
                        }

                        if (getPruning(prunTable, idxSym) == checkValue) {
                            setPruning(prunTable, idxSym, xorValue);
                        }
                    }

                    if (inverseSearch) {
                        break;
                    }
                }
            }
        }
    }

    // ---------- High-level helpers to build specific pruning tables ----------

    static void initTwistFlipPrun(boolean fullInit) {
        initRawSymPrun(
                TwistFlipPrun,
                null,
                null,
                TwistMove,
                Cubie.SymStateTwist,
                Constants.PRUNFLAG_TWIST_FLIP,
                fullInit
        );
    }

    static void initSliceTwistPrun(boolean fullInit) {
        initRawSymPrun(
                UDSliceTwistPrun,
                UDSliceMove,
                UDSliceConj,
                TwistMove,
                Cubie.SymStateTwist,
                Constants.PRUNFLAG_SLICE_TWIST,
                fullInit
        );
    }

    static void initSliceFlipPrun(boolean fullInit) {
        initRawSymPrun(
                UDSliceFlipPrun,
                UDSliceMove,
                UDSliceConj,
                FlipMove,
                Cubie.SymStateFlip,
                Constants.PRUNFLAG_SLICE_FLIP,
                fullInit
        );
    }

    static void initMCPermPrun(boolean fullInit) {
        initRawSymPrun(
                MCPermPrun,
                MPermMove,
                MPermConj,
                CPermMove,
                Cubie.SymStatePerm,
                Constants.PRUNFLAG_MC_PERM,
                fullInit
        );
    }

    static void initPermCombPPrun(boolean fullInit) {
        initRawSymPrun(
                EPermCCombPPrun,
                CCombPMove,
                CCombPConj,
                EPermMove,
                Cubie.SymStatePerm,
                Constants.PRUNFLAG_EPERM_COMBP,
                fullInit
        );
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
        this.tsym  = node.tsym;
        this.flip  = node.flip;
        this.fsym  = node.fsym;
        this.slice = node.slice;
        this.prun  = node.prun;

        if (PhaseSolver.USE_CONJ_PRUN) {
            this.twistc = node.twistc;
            this.flipc  = node.flipc;
        }
    }

    //Phase 1 pruning

    boolean setPrun(Cubie cc, int depthLimit) {
        twist = cc.getTwistSym();
        flip  = cc.getFlipSym();

        tsym  = twist & 7;
        twist >>>= 3;

        prun = PhaseSolver.USE_TWIST_FLIP_PRUN
                ? getPruning(
                        TwistFlipPrun,
                        (twist << 11) | Cubie.FlipS2RF[flip ^ tsym]
                  )
                : 0;

        if (prun > depthLimit) {
            return false;
        }

        fsym = flip & 7;
        flip >>>= 3;

        slice = cc.getUDSlice();

        int idxST = twist * Constants.N_SLICE + UDSliceConj[slice][tsym];
        int idxSF = flip  * Constants.N_SLICE + UDSliceConj[slice][fsym];

        prun = Math.max(
                prun,
                Math.max(
                        getPruning(UDSliceTwistPrun, idxST),
                        getPruning(UDSliceFlipPrun, idxSF)
                )
        );

        if (prun > depthLimit) {
            return false;
        }

        if (PhaseSolver.USE_CONJ_PRUN) {
            Cubie conj = new Cubie();
            Cubie.CornerConj(cc, 1, conj);
            Cubie.EdgeConj(cc, 1, conj);

            twistc = conj.getTwistSym();
            flipc  = conj.getFlipSym();

            int tSym = twistc & 7;
            int tIdx = twistc >>> 3;

            int key = (tIdx << 11) | Cubie.FlipS2RF[flipc ^ tSym];

            prun = Math.max(
                    prun,
                    getPruning(TwistFlipPrun, key)
            );
        }

        return prun <= depthLimit;
    }

    void calcPruning(boolean isPhase1) {
        int idxST = twist * Constants.N_SLICE + UDSliceConj[slice][tsym];
        int idxSF = flip  * Constants.N_SLICE + UDSliceConj[slice][fsym];

        int base = Math.max(
                getPruning(UDSliceTwistPrun, idxST),
                getPruning(UDSliceFlipPrun, idxSF)
        );

        int conjVal = 0;
        if (PhaseSolver.USE_CONJ_PRUN) {
            int tSym = twistc & 7;
            int tIdx = twistc >>> 3;
            int key = (tIdx << 11) | Cubie.FlipS2RF[flipc ^ tSym];
            conjVal = getPruning(TwistFlipPrun, key);
        }

        int tfVal = 0;
        if (PhaseSolver.USE_TWIST_FLIP_PRUN) {
            int key = (twist << 11) | Cubie.FlipS2RF[(flip << 3) | (fsym ^ tsym)];
            tfVal = getPruning(TwistFlipPrun, key);
        }

        prun = Math.max(base, Math.max(conjVal, tfVal));
    }


    int pruningMoves(Coordinates node, int move, boolean isPhase1) {
        slice = UDSliceMove[node.slice][move];

        int flipMoveIndex = Cubie.Sym8Move[(move << 3) | node.fsym];
        flip = FlipMove[node.flip][flipMoveIndex];
        fsym = (flip & 7) ^ node.fsym;
        flip >>>= 3;

        int twistMoveIndex = Cubie.Sym8Move[(move << 3) | node.tsym];
        twist = TwistMove[node.twist][twistMoveIndex];
        tsym = (twist & 7) ^ node.tsym;
        twist >>>= 3;

        int idxST = twist * Constants.N_SLICE + UDSliceConj[slice][tsym];
        int idxSF = flip  * Constants.N_SLICE + UDSliceConj[slice][fsym];

        int base = Math.max(getPruning(UDSliceTwistPrun, idxST), getPruning(UDSliceFlipPrun, idxSF));

        int tfVal = 0;
        if (PhaseSolver.USE_TWIST_FLIP_PRUN) { 
            int key = (twist << 11) | Cubie.FlipS2RF[(flip << 3) | (fsym ^ tsym)]; tfVal = getPruning(TwistFlipPrun, key);
        }

        prun = Math.max(base, tfVal);
        return prun;
    }

    int pruningMovesConj(Coordinates node, int move) {
        int conjMove = Cubie.SymMove[3][move];

        int flipStateIndex = node.flipc >>> 3;
        int flipSymBits    = node.flipc & 7;

        int flipMoveIndex = Cubie.Sym8Move[(conjMove << 3) | flipSymBits];
        flipc = FlipMove[flipStateIndex][flipMoveIndex] ^ flipSymBits;

        int twistStateIndex = node.twistc >>> 3;
        int twistSymBits    = node.twistc & 7;

        int twistMoveIndex = Cubie.Sym8Move[(conjMove << 3) | twistSymBits];
        twistc = TwistMove[twistStateIndex][twistMoveIndex] ^ twistSymBits;

        int tSym = twistc & 7;
        int tIdx = twistc >>> 3;

        int key = (tIdx << 11) | Cubie.FlipS2RF[flipc ^ tSym];
        return getPruning(TwistFlipPrun, key);
    }
}



"""Proposed Psuedo-code for the future implementation of the secure auction.
This structure proposes the use of methods which will be used to more easily divide and facilitate the various functions
of the code. To reiterate the steps of the initial secure auction:
(i) At the start of each round, there is a current set of prices, with the smallest
one equal to 0.
(ii) We construct the preferred-seller graph and check whether there is a perfect
matching
(iii) If there is, we’re done: the current prices are market-clearing.
(iv) If not, we find a constricted set of buyers S and their neighbors N(S).
(v) Each seller in N(S) (simultaneously) raises his price by one unit.
(vi) If necessary, we reduce the prices — the same amount is subtracted from
each price so that the smallest price becomes zero.
(vii) We now begin the next round of the auction, using these new prices.

README: THE STEP NUMBERS PROVIDED BELOW DO NOT LINE UP WITH THE NUMBERS ABOVE. THIS IS FOR REFERENCE AND SIMPLICITY.
WE SHOULD FOLLOW THE ABOVE STEPS FOR THE ALGORITHM, [NOT] FOR ORGANIZATION OF THE CODE.

DEVELOPMENT OF THIS STRUCTURE IS STILL BEING CONSIDERED. DETAILS MAY CHANGE. Let's make sure to keep up to date!

FURTHER CONSIDERATION: Let us come up with a more consistent naming schema. I have proposed here some changes to names
from our original code to make sure the method names are consistent."""
# !/usr/bin/env python3
import asyncio
import logging
import numpy as np
from mpyc.runtime import mpc
from itertools import chain, combinations
#import robot_PID.Robot_PID
import pycreate2
from functools import partial
from time import sleep
import scripts.roomba as controller
import scripts.goal_setter as setter

secflot = mpc.SecFxp(64)
secint = mpc.SecInt(64)
p_ID = mpc.pid


@mpc.coroutine
async def linear_solve(A, B):
    secnum = type(A[0][0])
    d, e = len(A), len(B[0])
    await mpc.returnType(secnum, d * e + 1)

    R, detR = random_matrix_determinant(secnum, d)
    RA = mpc.matrix_prod(R, A)
    RA = await mpc.output([a for row in RA for a in row], raw=True)
    RA = np.reshape(RA, (d, d))
    RB = mpc.matrix_prod(R, B)
    RB = await mpc.gather(RB)  # NB: RB is secret-shared

    invA_B, detRA = bareiss(secnum.field, np.concatenate((RA, RB), axis=1))
    detA = detRA / detR
    adjA_B = [secnum(a) * detA for row in invA_B for a in row]
    return adjA_B + [detA]


def bareiss(Zp, A):
    """Bareiss-like integer-preserving Gaussian elimination adapted for Zp.
    Using exactly one modular inverse in Zp per row of A.
    """
    p = Zp.modulus
    d, d_e = A.shape  # d by d+e matrix A

    # convert A elementwise from Zp to int
    for i in range(d):
        for j in range(d_e):
            A[i, j] = A[i, j].value

    # division-free Gaussian elimination
    for k in range(d):
        for i in range(k+1, d):
            for j in range(k+1, d_e):
                A[i, j] = (A[k, k] * A[i, j] - A[k, j] * A[i, k]) % p

    # back substitution
    for i in range(d-1, -1, -1):
        Zp_value = Zp(A[i, i])
        '''if Zp is 0, division by zero is prohibited'''
        if Zp_value!=0:
            inv = (1 / Zp(A[i, i])).value

            if i < d-2:
                A[i, i] = inv  # keep reciprocal for determinant
            for j in range(d, d_e):
                s = A[i, j]
                for k in range(i+1, d):
                    s -= A[i, k] * A[k, j]
                s %= p
                A[i, j] = (s * inv) % p

    # postponed division for determinant
    inv = 1
    det = A[d-1, d-1]
    for i in range(d-2):
        inv = (inv * A[i, i]) % p
        det = (det * inv) % p

    return A[:, d:], det

def random_matrix_determinant(secnum, d):
    d_2 = d * (d-1) // 2
    L = np.diagflat([secnum(1)] * d)
    L[np.tril_indices(d, -1)] = mpc._randoms(secnum, d_2)
    L[np.triu_indices(d, 1)] = [secnum(0)] * d_2
    diag = mpc._randoms(secnum, d)
    U = np.diagflat(diag)
    U[np.tril_indices(d, -1)] = [secnum(0)] * d_2
    U[np.triu_indices(d, 1)] = mpc._randoms(secnum, d_2)
    R = mpc.matrix_prod(L.tolist(), U.tolist())
    detR = mpc.prod(diag)  # detR != 0 with overwhelming probability
    return R, detR

async def is_match_matrix(assignments,subtraction_results_l):
    k = 4
    n = len(assignments)

    try:
        for c in range(k):

            matrix_v = await N2random_matrix(assignments, n)
            matrix_B = [0 for i in range(n)]
            matrix_B = [secflot(b) for b in matrix_B]
            matrix_B = mpc.convert(matrix_B, secflot)
            B = [[None] * 1 for _ in range(n)]

            index = 0
            for i in range(n):
                for j in range(1):
                    B[i][j] = matrix_B[index]
                    index += 1

            '''if det(A) is 0 exit the loop and break the tie.'''

            w_det = linear_solve(matrix_v, B)
            w_det = await mpc.output(w_det)

            if w_det[-1]== 0:
                print("no match!")
                return 0
            else:
                return (-1)
    except ImportError:
        pass

async def N2random_matrix(matrix, n):
    assi_matrix = [[None] for a in range(len(matrix))]
    secnum = type(matrix[0][0])

    index = 0
    for x in matrix:
        assi_matrix[index]=x
        index +=1

    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            #if matrix[i][j] == secint(1):
            if await mpc.output(mpc.eq(await mpc.output(assi_matrix[i][j]), secint(1))):
                #temp = np.random.randint(1, 2 * n)
                randomnum= mpc.random.randint(secnum,1, 2*n)
                assi_matrix[i][j] = randomnum
    return assi_matrix

def subtration(evaluations,prices)->asyncio.Future:
    """The subtraction should take place here. Preferably, we should also mix this in with the assignment matrix
    assignment matching algorithm. We should discuss this jointly."""

    subtraction_results = mpc.vector_sub(evaluations,prices)
    subtraction_results = mpc.input(subtraction_results)
    return subtraction_results

## ALL METHODS FOR THIS LOOP WILL BE LISTED BENEATH THIS ONE. EXTERNAL METHODS, LIKE subtraction(), ARE ABOVE. ##
"""WHY UNDERSCORE match_loop? As the main portion of our code, it differentiates it from utility methods like findEdges
or createPowerset. Please let me know your opinion on this."""

async def match_loop(valuationsmatrix,subtraction_results,prices_l,p_ID):

    auction_round = 0

    while True:
        '''Step 1)
        Find local maximum and create the assignment matrix. Check the perfect catch of this assignment 
        In case the current assignment is a perfect mach then stop. Otherwise move to step 2'''
        #print('round:', auction_round)

        reevaluations = prices_l
        logging.info("Local max found")
        localmax = [secint(0) for x in subtraction_results]
        for x in range(len(subtraction_results)):
            localmax[x] = await mpc.output(mpc.argmax(subtraction_results[x])[1])
        print()

        assignemnt_matrix = [secint(0)] * len(subtraction_results[p_ID])


        assignemnt_matrix = mpc.input(assignemnt_matrix)
        decoded_assigned_matrix = [None] * len(assignemnt_matrix)
        '''create assignment row to form assignment matrix'''
        for ID in range(len(subtraction_results)):
            row = await mpc.output(subtraction_results[ID])
            row = [round(num, 1) for num in row]
            #print(row)
            '''place 1s in the position where max exists in the matrix'''
            for x in range(len(row)):
                if await mpc.output(mpc.eq(subtraction_results[ID][x], localmax[ID])):
                    assignemnt_matrix[ID][x] = secint(1)

        for r in range(len(assignemnt_matrix)):
            tx = await mpc.output(assignemnt_matrix[r])
            tx = [round(num, 1) for num in tx]
            print(tx)
            decoded_assigned_matrix[r]=tx
        '''Check if this assignment matrix is perfect mach'''
        match = await is_match_matrix(assignemnt_matrix,subtraction_results)

        if match==0:

            """Step 2
            We need to find the constricted set. This is composed of multiple steps. First, we need to find all the matching
            edges in some form. I use a coordinate system for this, such as (0,1), or [1,3]. We should prefer lists so that
            we can create seclists to keep the values hidden."""
            subList = []
            '''find the min increament rate from the matrix min(max1-max2)+0.1  '''
            for item in range(len(subtraction_results)):
                sorted_row = mpc.sorted(subtraction_results[item])
                x1 = sorted_row[-1]
                x2 = sorted_row[-2]
                sub = mpc.abs(mpc.sub(x1, x2))
                subList.append(sub)
            incrementFactor = mpc.min(subList)

            if await mpc.output(mpc.lt(incrementFactor, secflot(1.0))):
                incrementFactor = secflot(1.0)


            edges = await findEdges(assignemnt_matrix)
            #TODO: need to chenge deleteDuplicates to secured approch
            matchBuyers = await deleteDuplicates(edges[1])
            # TODO: need to change create_powerset to secured approch
            powerset = list(create_powerset(matchBuyers))
            neighborSet =await cardinalities(powerset, edges[0])
            """Step 3
            We need to raise prices based on the Max1 - Max2 principle."""

            reevaluations = await raisePrices(prices_l,incrementFactor,neighborSet)

            logging.info("prices updated")
            reevaluations = await mpc.output(reevaluations)
            reevaluations = [round(num, 1) for num in reevaluations]
            print(reevaluations)

            print()

            '''subtract valuations from new prices and print prices'''
            logging.info("new utilities calculated")
            stype = type(subtraction_results[0][0])
            comnum = mpc.convert(secint(0),stype)
            for r in range(len(subtraction_results)):
                subtraction_results[r] = mpc.vector_sub(valuationsmatrix[r], prices_l)
                for index in range(len(subtraction_results[r])):
                    if await mpc.output(mpc.lt(subtraction_results[r][index], comnum)):
                        subtraction_results[r][index] = secflot(0)

            for vp in subtraction_results:
                vp = await mpc.output(vp)
                vp = [round(num, 1) for num in vp]
                print(vp)

            auction_round += 1
        else:

            print("match!")
            #reevaluations = await mpc.output(assignemnt_matrix)
            #reevaluations = [round(val, 1) for val in reevaluations]
            #print(reevaluations)
            break  # False

    """Step 4
    Repeat the loop."""
    #TODO: before returing revaluations to main, check clum wise for any conflict and retun the a list of pos
    # of assigned passenger ex [0,1,0] ==> [2] which is the pos of assignment
    robot_assignment = np.asarray(decoded_assigned_matrix)
    robot_assignment = robot_assignment.tolist()
    for row in range(len(robot_assignment)):
            if robot_assignment[row].count(1)==1:
                curRow = robot_assignment[row]
                onePos = curRow.index(1)
                for row_c in range(len(robot_assignment)):
                    if row_c != row:
                        robot_assignment[row_c][onePos] = 0

    for row in range(len(robot_assignment)):
            if [item[row] for item in robot_assignment].count(1)>1:
                for r in range(len(robot_assignment)):
                    curRow = robot_assignment[r]
                    onePos = curRow.index(1)
                    for r_c in range(len(robot_assignment)):
                            if r_c != r:
                                robot_assignment[r_c][onePos] = 0
    print()
    print("Final Assignment Matrix:")
    for i in robot_assignment:
        print(i)
    print()
    assigned_pos= robot_assignment[p_ID].index(1)
    print("my assigned pos",assigned_pos)
    return assigned_pos

async def findEdges(assignemnt_matrix_l):
    # Finds and return coordinate edges.

    '''find the positions (x,y) where buyers conflicts in assignment matrix'''
    edges = []
    tied_sellers =[]
    #row_sellers_conflict = []
    for row in range(len(assignemnt_matrix_l)):
        for cell in range(len(assignemnt_matrix_l[row])):
            #print(await mpc.output(assignemnt_matrix_l[x][y]))
            if not await mpc.output(mpc.eq(assignemnt_matrix_l[row][cell], secint(0))):
                edges.append([secint(row), secint(cell)])
                tied_sellers.append(secint(row))

    '''convert indexes from secInt To secFlex'''
    #row_sellers_conflict = mpc.convert(row_sellers_conflict,secflot)

    return [edges,tied_sellers]
async def deleteDuplicates(edges_l):
    # Deletes duplicated matches for the creation of the powerset.

    found = set([])
    keep = []

    for item in edges_l:
        if await mpc.output(item) not in found:
            found.add(await mpc.output(item))
            keep.append(await mpc.output(item))

    return keep
def create_powerset(values):
    xs = list(values)
    # note we return an iterator rather than a list
    return chain.from_iterable(combinations(xs, n) for n in range(len(xs) + 1))
async def cardinalities(powerset, edges):

    for buyerSet in powerset:
        sellers = set()
        for buyer in buyerSet:
            for match in edges:
                if await mpc.output(match[0]) == buyer:
                    sellers.add(await mpc.output(match[1]))
        if len(sellers) < len(buyerSet):
            return list(sellers)
    return -1
async def raisePrices(prices,incrementFactor,neighborSet):
    # This is where we should implement the raising of the prices when the matches are found.

    '''update the price on all price lists belong to each player'''
    #TODO: Need to change price on only one price in the list, rather than reflected on the whole list conflect.
    factor = await mpc.output(incrementFactor)
    #print("incrementFactor", await mpc.output(incrementFactor))
    print("incrementFactor", round(factor,1))

    '''first start with the changes caused by row conflict'''

    if neighborSet != -1:
        for p in neighborSet:
            prices[int(p)] += incrementFactor
    else:
        #for p in indeces:
        #    prices[indeces[p]] += incrementFactor
        exit(-1)
    return prices

    ## End list of match_loop methods. Please attach more as you feel is fit. ##
async def CalculateDistances(curPassenger,autoVehicles):
    #Holder
    distances = []

    for veh in autoVehicles:
        sub_xy = mpc.vector_sub(veh,curPassenger)
        #print("sub_xy: ", await mpc.output(sub_xy))
        dot_prod= mpc.in_prod(sub_xy, sub_xy)
        #print("dot_prod: ", await mpc.output(dot_prod))
        sq= mpc.statistics._fsqrt(dot_prod)
        #print("sq: ", await mpc.output(sq))
        distances.append(sq)

    return distances

@mpc.coroutine
async def main()-> asyncio.Future:


    '''
    MPYC auction entry point
    '''
    await mpc.start()

    n = 3
    prices = [0.0 for i in range(n)]
    prices = [secflot(p) for p in prices]

    valuations = []
    #passengers_pos in the grids prospective [[0,2][1,2][2,2]]
    #TODO: get these coordinates from the ros frame

    passengers_tags = ["tag20","tag22","tag23"]
    #passengers_tags = ["tag20"]
    passengers_pos = []

    for goaltag in passengers_tags:
        #goal_state = partial(setter.state_estimator_d, goaltag)
        goal_state = setter.state_estimator_d(goaltag)
        #pos_goal = goal_state()
        print("pos_goal", goal_state)
        goal_x = goal_state.transform.translation.x
        goal_y = goal_state.transform.translation.y
        goal_w = goal_state.transform.rotation.w

        passengers_pos.append([goal_x,goal_y,goal_w])

    print("passengers coordinates:")
    for passe in passengers_pos:
        print(passe)

    passengers = [[secflot(i) for i in item] for item in passengers_pos]
    passengers = [[mpc.convert(i, secflot) for i in l] for l in passengers]

    # TODO: get these coordinates from the ros frame
    autoVehicle_tag = "tag21"
    autoVehicle_state = partial(setter.state_estimator_d, autoVehicle_tag)
    pos_autoVehicle = autoVehicle_state()
    autoVehicle_x = pos_autoVehicle.transform.translation.x
    autoVehicle_y = pos_autoVehicle.transform.translation.y
    autoVehicle_w = pos_autoVehicle.transform.rotation.w

    pos = [autoVehicle_x, autoVehicle_y, autoVehicle_w]

    print("my coordinates:")
    print(pos[:2])

    autoVehicles = pos[:2]
    autoVehicles = [secflot(b) for b in autoVehicles]
    autoVehicles = mpc.input(autoVehicles)

    logging.info("Calculating distances securely and setting preferences")
    for curPassenger in passengers:
        distances = await CalculateDistances(curPassenger[:2], autoVehicles)
        valuations.append(distances)

    valuations = np.asarray(valuations)
    valuations = np.ndarray.tolist(valuations)
    #print("leng of valuation list:",len(valuations[p_ID]))
    subtraction_results = [[None]] * len(valuations[p_ID])

    for eval in range(len(valuations)):
        subtraction_results[eval] = mpc.vector_sub(valuations[eval], prices)

    valuationsmatrix = [[None]] * len(valuations[p_ID])
    for eval in range(len(valuations)):
        valuationsmatrix[eval] = mpc.vector_sub(valuations[eval], prices)

    for vp in subtraction_results:
        vp = await mpc.output(vp)
        vp = [round(num,1) for num in vp]
        print(vp)

    assignment = await match_loop(valuationsmatrix,subtraction_results,prices,p_ID)
    await mpc.shutdown()

    '''
              P1, P2, P3
    robot 1 [ R1P1, R1P2, R1P3]
    robot 2 [ R2P1, R2P2, R2P3]
    robot 3 [ R3P1, R3P2, R3P3]
    '''
    robot_assignment = passengers_pos[assignment]
    print("assigned passenger pos:",robot_assignment)

    #setter.goal_setter(robot_assignment[0],robot_assignment[1],robot_assignment[2])

mpc.run(main())



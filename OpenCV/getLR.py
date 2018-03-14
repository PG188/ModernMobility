def getLR():

    cnt = [(0,1), (-4, 6), (2, 3), (-2, 5)]
    L = 0
    R = 0
    
    for i in range(len(cnt)):
        if cnt[i][0] < L:
            L = cnt[i][0]
        if cnt[i][0] > R:
            R = cnt[i][0]

    return L, R
        
        

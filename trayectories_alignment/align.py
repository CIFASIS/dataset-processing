"""Align two trajectories using the method of Horn (closed-form).

Input:
model -- first trajectory (3xn)
data -- second trajectory (3xn)

Output:
rot -- rotation matrix (3x3)
trans -- translation vector (3x1)
trans_error -- translational error per point (1xn)

"""
numpy.set_printoptions(precision=3,suppress=True)
model_zerocentered = model - model.mean(1)
data_zerocentered = data - data.mean(1)

W = numpy.zeros( (3,3) )
for column in range(model.shape[1]):
    W += numpy.outer(model_zerocentered[:,column],data_zerocentered[:,column])
U,d,Vh = numpy.linalg.linalg.svd(W.transpose())
S = numpy.matrix(numpy.identity( 3 ))
if(numpy.linalg.det(U) * numpy.linalg.det(Vh)<0):
    S[2,2] = -1
rot = U*S*Vh
trans = data.mean(1) - rot * model.mean(1)

model_aligned = rot * model + trans
alignment_error = model_aligned - data

trans_error = numpy.sqrt(numpy.sum(numpy.multiply(alignment_error,alignment_error),0)).A[0]
    
print rot
print trans



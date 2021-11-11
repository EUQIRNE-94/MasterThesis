function L = laplaciano(A)

L = diag(sum(A,2)) + (-A + diag(diag(A)));

end
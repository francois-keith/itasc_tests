#/bin/bash

function testcpf ()
{
	echo 
	echo ' ****** testing '$1' ****** '
	echo 
	cd cpf
	rm tester.cpf
	ln -s $1  tester.cpf
	cd ..
	./run.sh
}

testcpf 01-equality.cpf
testcpf 02-inequality.cpf
testcpf 03-test_in_eq.cpf
testcpf 04-test_in_eq.cpf
testcpf 05-test_in_eq.cpf
testcpf 06-equality-wq.cpf
testcpf 07-equality-wy.cpf

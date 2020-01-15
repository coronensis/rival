
# Reset, etc. vectors go here
        .section "vectors"
reset:   j      _start

	.text
	.globl	_start


# Initialize and jump into main
_start:
	# the stack grows downwards so start at the top of (internal) memory
	la	sp, 0x7FF
	call	main
	j	_start


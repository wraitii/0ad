var test = 0;

function incrementTest()
{
	test += 1;
}

async function waitAndIncrement(promise)
{
	await promise;
	incrementTest();
}

function runTest()
{
	var rsv;
	let prom = new Promise((resolve, reject) => {
		incrementTest();
		rsv = resolve;
	});
	waitAndIncrement(prom);
	TS_ASSERT_EQUALS(test, 1);
	rsv();
	// At this point, waitAndIncrement is still not run, but is now free to run.
	TS_ASSERT_EQUALS(test, 1);
}

runTest();

function endTest()
{
	TS_ASSERT_EQUALS(test, 2);
}

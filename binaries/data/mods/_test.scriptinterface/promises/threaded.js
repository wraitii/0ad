var res = 0;

async function runSquare() {
	res = await Engine.AsyncSquare(5);
}

runSquare();
// At this point the function is not run.
TS_ASSERT_EQUALS(res, 0);

function endTest() {
	TS_ASSERT_EQUALS(res, 5 * 5);
}

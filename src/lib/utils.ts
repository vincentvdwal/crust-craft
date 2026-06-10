export const pad = (n: string | number, amount = 2) => {
	return String(n).padStart(amount, '0');
};

/**
 * @module ringbuffer
 * RingBuffer class.
 */

/**************************************************************************
*   Copyright 2023 Christoph Schmidtmeier                                 *
*   Robotics Erlangen e.V.                                                *
*   http://www.robotics-erlangen.de/                                      *
*   info@robotics-erlangen.de                                             *
*                                                                         *
*   This program is free software: you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation, either version 3 of the License, or     *
*   any later version.                                                    *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU General Public License for more details.                          *
*                                                                         *
*   You should have received a copy of the GNU General Public License     *
*   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
**************************************************************************/

export class RingBuffer<T> {
	private _buffer: (T | undefined)[] = [];
	private _size: number = 0;
	private _length: number = 0;
	private _read: number = 0;
	private _write: number = 0;

	/**
	 * Creates a ringbuffer of size, prepopulated with values
	 * @param size - The size of the ringbuffer
	 * @param values - optional, initial content of the ringbuffer
	 */
	constructor(size: number, values: T[] = []) {
		if (size <= 0) {
			throw Error(`Trying to create a ringbuffer of size ${size}`);
		}
		this._size = size;
		this._length = values.length;
		this._read = 0;
		this._write = values.length;

		// creates a buffer of size from values, copying the list
		if (values.length > size) {
			throw Error(`Too many initial values for ringbuffer, got ${values} as initial values for a ringbuffer of size ${size}`);
		}
		this._buffer = [...values];
		this._buffer.length = size;
	}

	/**
	 * Increments a number modulo the size of this ringbuffer
	 * @param x - the number to increment
	 * @param y - the increment, defaults to 1
	 * @returns (x + y) % size of the ringbuffer
	 */
	private _inc(x: number, y: number = 1): number {
		return (x + y) % this._size;
	}

	/**
	 * Run every time the queue is cleared
	 */
	protected _onClear() {}

	/**
	 * Run every time an element is removed
	 * @param element - The removed element
	 */
	protected _onRemove(element: T) {}

	/**
	 * Run every time an element is added
	 * @param element - The added element
	 */
	protected _onPut(element: T) {}

	/**
	 * Gets the maximum number of elements that can be in the ringbuffer at the same time
	 * @returns The size of the ringbuffer
	 */
	public size(): number {
		return this._size;
	}

	/**
	 * Gets the current number of elements that are in the ringbuffer
	 * @returns The length of the ringbuffer
	 */
	public length(): number {
		return this._length;
	}

	/**
	 * Returns true if the ringbuffer is emtpy
	 * @returns true if the ringbuffer is emtpy
	 */
	public isEmpty(): boolean {
		return this._length === 0;
	}

	/**
	 * Returns true if the ringbuffer is full
	 * @returns true if the ringbuffer is full
	 */
	public isFull(): boolean {
		return this._length === this._size;
	}

	/**
	 * Empties the ringbuffer
	 */
	public clear() {
		this._length = 0;
		this._buffer.length = 0;
		this._buffer.length = this._size;
		this._onClear();
	}

	/**
	 * Removes a values from the ringbuffer if possible, else returns undefined
	 * @returns The next value from the ringbuffer or undefined if the buffer is empty
	 */
	public removeOrUndefined(): T | undefined {
		if (this._length === 0) {
			return undefined;
		}

		const x = this._buffer[this._read]!;
		this._read = this._inc(this._read);
		this._length -= 1;
		this._onRemove(x);
		return x;
	}

	/**
	 * Removes a value from the ringbuffer if possible, else throws an error
	 * @returns The next value from the ringbuffer
	 */
	public remove(): T {
		if (this._length === 0) {
			throw Error("remove called on empty ringbuffer");
		}
		return this.removeOrUndefined()!;
	}

	/**
	 * Puts a value into the ringbuffer, possibly overwriting the oldest value if the ringbuffer is full
	 * @param x - The value to store
	 * @returns The overwritten value or undefined if the ringbuffer wasnt full
	 */
	public putOrReplace(x: T): T | undefined {
		const ret = (this._length === this._size) ? this.removeOrUndefined()! : undefined;

		this._buffer[this._write] = x;
		this._write = this._inc(this._write);
		this._length += 1;
		this._onPut(x);
		return ret;
	}

	/**
	 * Puts a value into the ringbuffer if possible, else throws an error
	 * @param x - The value to store
	 */
	public put(x: T) {
		if (this._length === this._size) {
			throw Error("put called on full ringbuffer");
		}
		this.putOrReplace(x);
	}

	/**
	 * Returns the next value to be read from the ringbuffer **after
	 * i remove operations** without removing it or undefined if
	 * the buffer is emtpy by then
	 * @param i - how many elements to skip, defaults to 0
	 * @returns The value or undefined if the buffer is empty
	 */
	public peekOrUndefined(i: number = 0): T | undefined {
		if (i < 0) {
			throw Error(`peekOrUndefined(${i}) called with negative argument`);
		}
		return i < this._length ? this._buffer[this._inc(this._read, i)] : undefined;
	}

	/**
	 * Returns the next value to be read from the ringbuffer **after
	 * i remove operations** without removing it, or throws an error
	 * if the buffer is emtpy by then
	 * @returns The value
	 */
	public peek(i: number = 0): T {
		if (i >= this._length) {
			throw Error(`peek(${i}) called on ringbuffer of size ${this._size}`);
		}
		return this.peekOrUndefined(i)!;
	}

	/**
	 * Returns a shallow copy of all values currently in the buffer, in the order they were added
	 * @returns An array containing all values currently in the buffer
	 */
	public toArray(): T[] {
		if (this._read + this._length <= this._size) {
			return this._buffer
				.slice(this._read, this._read + this._length)
				.map((x) => x!);
		} else {
			return [
				...this._buffer.slice(this._read, this._size),
				...this._buffer.slice(0, this._read + this._length - this._size)
			].map((x) => x!);
		}
	}


	/**
	 * Returns a string representation of the ringbuffer, used for base/debug
	 * @returns A string representation of the ringbuffer
	 */
	public _toString() {
		return `RingBuffer(size=${this._size}, ${this.toArray()})`;
	}

	/**
	 * Returns a string representation of the ringbuffer
	 * @returns A string representation of the ringbuffer
	 */
	public toString() {
		return this._toString();
	}
}

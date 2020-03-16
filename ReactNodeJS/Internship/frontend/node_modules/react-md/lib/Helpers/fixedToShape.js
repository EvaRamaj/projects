'use strict';

Object.defineProperty(exports, "__esModule", {
  value: true
});

var _propTypes = require('prop-types');

var _propTypes2 = _interopRequireDefault(_propTypes);

function _interopRequireDefault(obj) { return obj && obj.__esModule ? obj : { default: obj }; }

exports.default = _propTypes2.default.oneOfType([_propTypes2.default.object, _propTypes2.default.shape({
  x: _propTypes2.default.object,
  y: _propTypes2.default.object
})]);
//# sourceMappingURL=fixedToShape.js.map
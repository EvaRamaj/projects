"use strict";

import React from 'react';
import { Toolbar, Button } from 'react-md';
import { withRouter } from 'react-router-dom'

import KebabMenu from './KebabMenu';


class Header extends React.Component {

    constructor(props) {
        super(props);
    }

    render() {
        return (
            <Toolbar>

            </Toolbar>
        );
    }
};
// colored
//nav={<Button onClick={() => this.props.history.push('/')} icon>home</Button>}
// title={this.props.name}
// actions={<KebabMenu id="toolbar-colored-kebab-menu" />}>
export default withRouter(Header);
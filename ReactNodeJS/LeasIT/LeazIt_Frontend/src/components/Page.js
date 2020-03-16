"use strict";

import React from 'react';

import Header from './Header';
import { Footer } from './Footer';


export default class Page extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            name: ''
        }
    }

    componentDidMount(){
       this.setState({
           name: document.name
       });
    }

    render() {
        return (
            <section>
                <Header title={this.state.name} />
                {this.props.children}
            </section>
        );
    }
}
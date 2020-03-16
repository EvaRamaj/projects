"use strict";

import React from 'react';
import { TableRow, TableColumn, FontIcon, Button, DataTable, TableBody, Cell  } from 'react-md';
import '../scss/custom.css';
import Page from "./Page";
import handShake from '../images/handShake.png';
import {withRouter} from 'react-router-dom';


class Homepage extends React.Component {

    constructor(props) {
        super(props);
        this.state = {str: ""};
        this.handleChange = this.handleChange.bind(this);
        this.handleSubmit = this.handleSubmit.bind(this);
    }
    handleChange(event){
        this.setState({str:event.target.value});
    }

    handleSubmit(event){
        event.preventDefault();
        this.props.onSubmit(this.state.str);
    }
    render() {
            return(
                <Page>
                    <div className="Home_text">
                        <a>LeazIt</a>
                    </div>
                    <div className="Home_text_details">
                        <h5>Rent everything in the blink of an eye! </h5>
                    </div>
                    <div className="row justify-content-md-center">
                        <div className="col-4">
                            <form onSubmit={this.handleSubmit}>
                                <input className="form-control mr-sm-2" type="search" onChange={this.handleChange}
                                       placeholder="Search for the item that you need" aria-label="Search"/>
                            </form>
                        </div>
                        <div className="col-1">
                            <Button className="btn btn-outline-success my-2 my-sm-0 search_button_front_page" style={{backgroundColor:"#009900"}} type="submit" raised>OK!</Button>
                        </div>
                    </div>
                    <div className="row justify-content-md-center">
                        <div className= "handShake">
                            <img className="img-fluid" src = {handShake} alt="Responsive image"/>
                        </div>
                    </div>
                </Page>
            );
        }
}
export default withRouter(Homepage);